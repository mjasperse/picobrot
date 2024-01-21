/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Modifications copyright (c) 2024 by Martijn Jasperse <m.jasperse@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>
#include "pico.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "config.h"
#include "QMI8658.h"

#if PICO_ON_DEVICE

#include "hardware/clocks.h"
#include "hardware/vreg.h"

#endif

CU_REGISTER_DEBUG_PINS(generation)


// *** CONFIGURATION OPTIONS ***
// Maximum number of iterations for the escape calculation
// Increase for more detail around the set edges at the expense of calculation time
#define MAX_ITERS 63//127//255

// Use floating-point arithmetic instead of fixed-point?
// Runs extremely slowly on the device, helpful for sanity-checking
//#define USE_FLOAT 1

// Special reduced-precision mode for much faster calculation speed at the expense of limiting the zoom level
#define FAST_AND_LOOSE_MODE 1

// Enable overclocking of sysclock for increased calculation and smoother rendering
#define TURBO_BOOST 0

// Number of fractional bits to use for fixed-point calculations
// Must be able to represent
#define FRAC_BITS 25u

// Apply colorization to the interior of the Mandelbrot set?
// Not necessarily a meaningful visualisation, but interesting to look at
#define COLOR_INTERIOR 1

// Define the framerate of updates to the LCD
// If not defined, writes to LCD at the end of each frame calculation
//#define FRAME_INTERVAL  1000000/60  // 60 fps

// Define the measurement rate for the accelerometer
#define ACCEL_INTERVAL  10000       // 10 ms
// Define the damping factor for accelerometer measurements
#define ACCEL_DAMPING   0.3
// Define the number of measurements for the initial value
#define ACCEL_INITIAL   10


#if USE_FLOAT
// Floating point mode for best resolution but slowest calculation speed
// 32-bit "float" offers very limited advantage over 32-bit fixed-point, so use 64-bit "double" instead

//typedef float fixed;
typedef double fixed;
static inline fixed float_to_fixed(float x) {
    return x;
}
static inline fixed fixed_mult(fixed a, fixed b) {
    return a*b;
}

#elif FAST_AND_LOOSE_MODE
// A variation of the fixed-point calculation where all intermediate calculations can be represented in I32.
// This means that zr,zi can be at most I16, so choose FRAC_BITS appropriately.
// This causes issue with truncation of the derivatives dx0_dx and dy0_dy, so we shift those up and then
// shift back down for the fractal calculation.

#undef FRAC_BITS            // ignore previous setting, for custom faster options
#define FRAC_BITS   13      // at most 13 bits, so we can represent x^2 in an I16
#define DERIV_SHIFT (1<<8)  // at most 8 bits, since it gets multiplied by DISPLAY_WIDTH=240

typedef int32_t fixed;      // in principle this could potentially be I16, but requires better handling of intermediate values

static inline fixed float_to_fixed(float x) {
    return (fixed) (x * (1u << FRAC_BITS));
}

static inline fixed fixed_mult(fixed a, fixed b) {
    // FRAC_BITS is chosen so that this multiplication does not overflow an I32
    return (a * b) >> FRAC_BITS;
}

#else
// Fixed-point calculation mode using 32-bit integers

typedef int32_t fixed;

static inline fixed float_to_fixed(float x) {
    return (fixed) (x * (float) (1u << FRAC_BITS));
}

#if !PICO_ON_DEVICE || (FRAC_BITS != 25)
// General-purpose fixed-point calculation, where the intermediate result must be handled as I64
static inline fixed fixed_mult(fixed a, fixed b) {
    int64_t r = ((int64_t) a) * b;
    return (int32_t) (r >> FRAC_BITS);
}
#else
// Since we're trying to go fast, do a better multiply of 32x32 preserving the bits we want
static inline fixed fixed_mult(fixed a, fixed b) {
    uint32_t tmp1, tmp2, tmp3;
    __asm__ volatile (
    ".syntax unified\n"
    "asrs   %[r_tmp1], %[r_b], #16 \n" // r_tmp1 = BH
    "uxth   %[r_tmp2], %[r_a]      \n" // r_tmp2 = AL
    "muls   %[r_tmp2], %[r_tmp1]   \n" // r_tmp2 = BH * AL
    "asrs   %[r_tmp3], %[r_a], #16 \n" // r_tmp3 = AH
    "muls   %[r_tmp1], %[r_tmp3]   \n" // r_tmp1 = BH * AH
    "uxth   %[r_b], %[r_b]         \n" // r_b = BL
    "uxth   %[r_a], %[r_a]         \n" // r_a = AL
    "muls   %[r_a], %[r_b]         \n" // r_a = AL * BL
    "muls   %[r_b], %[r_tmp3]      \n" // r_b = BL * AH
    "add    %[r_b], %[r_tmp2]      \n" // r_b = BL * AH + BH * AL
    "lsls   %[r_tmp1], #32 - 25    \n" // r_tmp1 = (BH * AH) >> (32 - FRAC_BITS)
    "lsrs   %[r_a], #16            \n" // r_a = (AL & BL) H
    "add    %[r_a], %[r_b]         \n"
    "asrs   %[r_a], #25- 16        \n" // r_a = (BL * AH + BH * AL) H | (AL & BL) H >> (32 - FRAC_BITS)
    "add    %[r_a], %[r_tmp1]      \n"
    : [r_a] "+l" (a), [r_b] "+l" (b), [r_tmp1] "=&l" (tmp1), [r_tmp2] "=&l" (tmp2), [r_tmp3] "=&l" (tmp3)
    :
    );
    return a;
}
#endif

#endif

#ifndef DERIV_SHIFT
// derivative calculation does not use locally-increased precision
#define DERIV_SHIFT  1
#endif

struct mutex frame_logic_mutex;
static void frame_update_logic();

static uint y;
static fixed x0, y0;
static fixed dx0_dx, dy0_dy;
static fixed max;
static bool params_ready;

static uint8_t use_accel = 0;
static float accel_meas[3] = {0,0,0};
static float accel_init[3] = {0,0,0};

static uint16_t framebuffer[DISPLAY_HEIGHT * DISPLAY_WIDTH];


#define PICO_SCANVIDEO_PIXEL_FROM_RGB8  RGB565
static uint16_t colors[16] = {
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(66, 30, 15),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(25, 7, 26),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(9, 1, 47),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(4, 4, 73),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(0, 7, 100),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(12, 44, 138),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(24, 82, 177),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(57, 125, 209),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(134, 181, 229),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(211, 236, 248),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(241, 233, 191),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(248, 201, 95),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(255, 170, 0),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(204, 128, 0),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(153, 87, 0),
        PICO_SCANVIDEO_PIXEL_FROM_RGB8(106, 52, 3),
};

static void generate_fractal_line(uint16_t *line_buffer, uint length, fixed mx, fixed my, fixed dmx_dx) {
    for (int x = 0; x < length; ++x) {
        int iters;
        fixed cr = mx / DERIV_SHIFT;
        fixed ci = my / DERIV_SHIFT;
        fixed zr = cr;
        fixed zi = ci;
        fixed xold = 0;
        fixed yold = 0;
        fixed z2;
        int period = 0;
        // visualise the Mandelbrot set using the escape algorithm
        // https://en.wikipedia.org/wiki/Plotting_algorithms_for_the_Mandelbrot_set#Optimized_escape_time_algorithms
        for (iters = 0; iters < MAX_ITERS; ++iters) {
            fixed zr2 = fixed_mult(zr, zr);
            fixed zi2 = fixed_mult(zi, zi);
            z2 = zr2 + zi2;
            if (z2 > max) {
                // trajectory diverging to infinity
                break;
            }
            fixed zrtemp = zr2 - zi2 + cr;
            zi = 2 * fixed_mult(zr, zi) + ci;
            zr = zrtemp;
            // detect whether the trajectory has converged to a fixed position
            if (zr == xold && zi == yold) {
                iters = MAX_ITERS + 1;
                break;
            }
            if (++period > 20) {
                // long trajectories might converge to a value other than the origin
                period = 0;
                xold = zr;
                yold = zi;
            }
        }
            // points within the set are black
        if (iters >= MAX_ITERS) {
#if COLOR_INTERIOR
            // did not escape, so colorize based on the trajectory
            iters = z2 >> 8;
            line_buffer[x] = colors[iters & 15u] & 0x1FF8;  // drop green
#else
            // just draw as black
            line_buffer[x] = 0;
#endif
        } else {
            // points outside the set are colored based on their escape time
            line_buffer[x] = colors[iters & 15u];
        }
        mx += dmx_dx;
    }
}

void output_frame_to_display()
{
    // customize this function for the particular attached display
#if CONFIG_NO_DMA
    LCD_1IN28_Display(framebuffer);
#else
    LCD_1IN28_DisplayDMA(framebuffer);
#endif
}


// "Worker thread" for each core
void __time_critical_func(render_loop)() {
    int core_num = get_core_num();
    printf("Rendering on core %d\n", core_num);
    while (true) {
        // Mutex protects shared state variables
        mutex_enter_blocking(&frame_logic_mutex);
        if (y == DISPLAY_HEIGHT) {
            params_ready = false;
#if !FRAME_INTERVAL && !CONFIG_NO_DMA
            // Kick off the DMA to display as soon as the frame is complete
            // Note that the mutex is held so this should NOT be a blocking transmission
            output_frame_to_display();
#endif
            frame_update_logic();
            y = 0;
        }
        // Move to the next line
        uint _y = y++;
        fixed _x0 = x0 * DERIV_SHIFT, _y0 = y0 * DERIV_SHIFT;
        fixed _dx0_dx = dx0_dx, _dy0_dy = dy0_dy;
        mutex_exit(&frame_logic_mutex);
        // Generate this line of the image
        generate_fractal_line(&framebuffer[_y * DISPLAY_WIDTH], DISPLAY_WIDTH, _x0, _y0 + _dy0_dy * _y, _dx0_dx);
    }
}

#if FRAME_INTERVAL
int64_t render_callback(alarm_id_t alarm_id, void *user_data) {
    // Device-specific frame display code
    output_frame_to_display();
    return FRAME_INTERVAL;
}
#endif

int64_t accelerometer_callback(alarm_id_t alarm_id, void *user_data) {
    // The accelerometer measurements are noisy, so apply some exponential damping
    float acc[3] = {0,0,0};
    QMI8658_read_acc_xyz(acc);
    for (int i = 0; i < 3; ++i)
        accel_meas[i] = ACCEL_DAMPING*acc[i] + (1-ACCEL_DAMPING)*accel_meas[i];
    return ACCEL_INTERVAL;
}

void core1_func() {
    // No additional config required
    render_loop();
}

int vga_main(void) {
    mutex_init(&frame_logic_mutex);
    frame_update_logic();

    // Both cores run the rendering loop, generating alternating lines
    multicore_launch_core1(core1_func);

#if PICO_ON_DEVICE
    #if FRAME_INTERVAL
        // Timer callback generates output to the device
        // Display sheering is expected because it is async with the fractal generation
        add_alarm_in_us(FRAME_INTERVAL, render_callback, NULL, true);
    #endif

    if (use_accel)
    {
        // Set a callback for reading the accelerometer
        add_alarm_in_us(ACCEL_INTERVAL, accelerometer_callback, NULL, true);
    }
#endif
    render_loop();
    return 0;
}

void __time_critical_func(frame_update_logic)() {
    static float offx = 0, offy = 0;
    if (!params_ready) {
        // Slowly zoom in the visualisation on adjacent frames
        static int foo = 0;
        float scale = DISPLAY_HEIGHT / 2;
        scale *= (7500 + (foo++) * (float) foo) / 10000.0f;
        if (use_accel)
        {
            // Allow the acceleration to move the visualization window
            // The accelerometer is measured in a timer callback to permit smoothing
            // Also scale by the zoom level to reduce jitter at high zoom
            offx += (accel_meas[1] - accel_init[1]) / scale;
            offy += (accel_meas[2] - accel_init[2]) / scale;
            // Window bounds - prevent going out of range
            if (offx < -1) offx = -1;
            else if (offx > 1.5) offx = 1.5;
            if (offy < -1.5) offy = -1.5;
            else if (offy > 1.5) offy = 1.5;
        }
        else
        {
            // Zoom towards the coordinates (-0.10, 0.92)
            offx = (MIN(foo, 200)) / 500.0f;
            offy = -(MIN(2*foo, 230)) / 250.0f;
        }
        x0 = float_to_fixed(offx + (-DISPLAY_WIDTH / 2) / scale - 0.5f);
        y0 = float_to_fixed(offy + (-DISPLAY_HEIGHT / 2) / scale);
        dx0_dx = float_to_fixed(DERIV_SHIFT / scale);
        dy0_dy = dx0_dx;
        max = float_to_fixed(4.f);
        params_ready = true;
    }
}

int main(void) {
    // Configure any optional over-clocking for improved frame-rate
    uint base_freq;
#if PICO_SCANVIDEO_48MHZ
    base_freq = 48000;
#else
    base_freq = 50000;
#endif
#if PICO_ON_DEVICE
#if TURBO_BOOST
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(10);
    set_sys_clock_khz(base_freq*6, true);
#else
    set_sys_clock_khz(base_freq * 3, true);
#endif
#endif

    // Initialise the device hardware [Waveshare RP2040-LCD-1.28]
    DEV_Module_Init();
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_InitDMA();
#ifdef ACCEL_INTERVAL
    use_accel = QMI8658_init();
    for (int i = 0; i < ACCEL_INITIAL; ++i)
        accelerometer_callback(0, NULL);
    accel_init[0] = accel_meas[0];
    accel_init[1] = accel_meas[1];
    accel_init[2] = accel_meas[2];
#endif

    // Re init uart now that clk_peri has changed
    setup_default_uart();
//    gpio_debug_pins_init();

    return vga_main();
}

