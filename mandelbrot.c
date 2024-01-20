/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
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

#if PICO_ON_DEVICE

#include "hardware/clocks.h"
#include "hardware/vreg.h"

#endif

CU_REGISTER_DEBUG_PINS(generation)

//#define USE_FLOAT 1

#if USE_FLOAT
//typedef float fixed;
typedef double fixed;
static inline fixed float_to_fixed(float x) {
    return x;
}
static inline fixed fixed_mult(fixed a, fixed b) {
    return a*b;
}
#else
#define FRAC_BITS 25u
typedef int32_t fixed;

static inline fixed float_to_fixed(float x) {
    return (fixed) (x * (float) (1u << FRAC_BITS));
}

#if !PICO_ON_DEVICE || (FRAC_BITS != 25)
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

#define max_iters 127//255

struct mutex frame_logic_mutex;
static void frame_update_logic();

static uint y;
static fixed x0, y0;
static fixed dx0_dx, dy0_dy;
static fixed max;
static bool params_ready;


static uint16_t framebuffer[DISPLAY_HEIGHT * DISPLAY_WIDTH];
//static uint16_t *framebuffer;

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

static void scanline(uint16_t *line_buffer, uint length, fixed mx, fixed my, fixed dmx_dx) {
    for (int x = 0; x < length; ++x) {
        int iters;
        fixed cr = mx;
        fixed ci = my;
        fixed zr = cr;
        fixed zi = ci;
        fixed xold = 0;
        fixed yold = 0;
        int period = 0;
        for (iters = 0; iters < max_iters; ++iters) {
            fixed zr2 = fixed_mult(zr, zr);
            fixed zi2 = fixed_mult(zi, zi);
            if (zr2 + zi2 > max) {
                break;
            }
            fixed zrtemp = zr2 - zi2 + cr;
            zi = 2 * fixed_mult(zr, zi) + ci;
            zr = zrtemp;
            if (zr == xold && zi == yold) {
                iters = max_iters + 1;
                break;
            }
            if (++period > 20) {
                period = 0;
                xold = zr;
                yold = zi;
            }
        }
        if (iters == max_iters + 1) {
            line_buffer[x] = 0;//x1f;
        } else {
            line_buffer[x] = iters == max_iters ? 0 : colors[iters & 15u];
        }
        mx += dmx_dx;
    }
}

void output_frame_to_display()
{
    // customize this function for the particular attached display
    LCD_1IN28_Display(framebuffer);
}


// "Worker thread" for each core
void __time_critical_func(render_loop)() {
    static uint32_t last_frame_num = 0;
    int core_num = get_core_num();
    printf("Rendering on core %d\n", core_num);
    while (true) {
        mutex_enter_blocking(&frame_logic_mutex);
        if (y == DISPLAY_HEIGHT) {
            params_ready = false;
            frame_update_logic();
            y = 0;
        }
        uint _y = y++;
        fixed _x0 = x0, _y0 = y0;
        fixed _dx0_dx = dx0_dx, _dy0_dy = dy0_dy;
        mutex_exit(&frame_logic_mutex);

        scanline(framebuffer + _y * DISPLAY_WIDTH, DISPLAY_WIDTH, _x0, _y0 + _dy0_dy * _y, _dx0_dx);
    }
}

int64_t timer_callback(alarm_id_t alarm_id, void *user_data) {
    output_frame_to_display();
    return 100;
}

void core1_func() {
    render_loop();
}

int vga_main(void) {
    mutex_init(&frame_logic_mutex);
    frame_update_logic();

    // Core 1 will wait for us to finish video setup, and then start rendering
    multicore_launch_core1(core1_func);

#if PICO_ON_DEVICE
    add_alarm_in_us(100, timer_callback, NULL, true);
#endif
    render_loop();
    return 0;
}

void __time_critical_func(frame_update_logic)() {
    if (!params_ready) {
        float scale = DISPLAY_HEIGHT / 2;
        static int foo;
        float offx = (MIN(foo, 200)) / 500.0f;
        float offy = -(MIN(foo, 230)) / 250.0f;
        scale *= (10000 + (foo++) * (float) foo) / 10000.0f;
        x0 = float_to_fixed(offx + (-DISPLAY_WIDTH / 2) / scale - 0.5f);
        y0 = float_to_fixed(offy + (-DISPLAY_HEIGHT / 2) / scale);
        dx0_dx = float_to_fixed(1.0f / scale);
        dy0_dy = float_to_fixed(1.0f / scale);
        max = float_to_fixed(4.f);
        params_ready = true;
    }
}

int main(void) {
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

    // Re init uart now that clk_peri has changed
    setup_default_uart();
//    gpio_debug_pins_init();

    return vga_main();
}
