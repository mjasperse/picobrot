cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

project(firmware)
pico_sdk_init()

add_executable(firmware
	mandelbrot.c
	DEV_Config.c
	LCD_1in28.c
)

target_compile_definitions(firmware PRIVATE
	PICO_ON_DEVICE=1
)

# enable usb output, disable uart output
pico_enable_stdio_usb(firmware 1)
pico_enable_stdio_uart(firmware 0)

# create map/bin/hex/uf2 file etc.
pico_set_program_name(firmware "Picobrot")
pico_set_program_version(firmware "0.1")

pico_add_extra_outputs(firmware)

# Add the standard library to the build
target_link_libraries(firmware pico_stdlib pico_multicore)

# Add the hardware libraries to the build
target_link_libraries(firmware
	hardware_spi
	hardware_i2c
	hardware_pwm
)

