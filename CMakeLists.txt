cmake_minimum_required(VERSION 3.20)

# Default to the 16 MB WeAct Studio RP2040 Core Board (LED on GP25, 16 MB QSPI).
set(PICO_BOARD weact_studio_rp2040_16mb CACHE STRING "Board type")

# Keep the SDK's stdio UART: on the working Pico build board_init() places
# UART0 TX on GP0 (nTRST) and RX on GP1 (nRESET), and UART0 TX idles high.
# That is what holds nTRST released from boot until JTAG claims the pins.
# DAP_SETUP() now also drives nTRST high explicitly (DAP_config.h).

include(pico_sdk_import.cmake)

project(JTAGprobe_smp_pio_std_weact C CXX ASM)

# tinyusb without an OS: every tinyusb call is serialised by the probe's own
# µOS++ mutex (src/probe_os.h), and none of tinyusb's own locking runs under
# the RTOS. Read when pico_sdk_init() adds tinyusb.
set(TINYUSB_OPT_OS OPT_OS_NONE)

pico_sdk_init()

# µOS++ III SMP: the kernel (micro-os-plus/micro-os-plus-iii, an INTERFACE
# library) and its RP2040 port (an INTERFACE library on top of it), linked
# below as micro-os-plus::rp2040 -- the way the micro-os-plus-iii-smp tests
# link micro-os-plus::cortexm.
add_subdirectory(micro-os-plus/micro-os-plus-iii-rp2040)

add_executable(JTAGprobe_smp_pio_std_weact

        src/DAP.c
        src/get_serial.c
        src/JTAG_DP.c
        src/SW_DP.c
        src/SWO.c
        src/DAP_vendor.c
        src/main.c
        src/usb_descriptors.c
        src/tusb_edpt_handler.c
        src/cdc_uart.c
        src/pio_jtag.c
        src/led.c
)

pico_generate_pio_header(JTAGprobe_smp_pio_std_weact ${CMAKE_CURRENT_LIST_DIR}/src/jtag.pio)

target_include_directories(JTAGprobe_smp_pio_std_weact PRIVATE

        CMSIS_6/CMSIS/Core/Include/
        include/
        )

target_compile_options(JTAGprobe_smp_pio_std_weact PRIVATE -Wall)

# src/ also holds cmsis-plus/os-app-config.h, the µOS++ configuration.
target_include_directories(JTAGprobe_smp_pio_std_weact PRIVATE src)

target_compile_definitions (JTAGprobe_smp_pio_std_weact PRIVATE
	PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1
	OS_USE_OS_APP_CONFIG_H
	# The kernel's malloc() already runs under the scheduler lock, across
	# both cores; the SDK's wrapper must not add a Pico mutex on top of it.
	PICO_USE_MALLOC_MUTEX=0
)


target_link_libraries(JTAGprobe_smp_pio_std_weact PRIVATE
        micro-os-plus::rp2040
        pico_stdlib
        pico_unique_id
        tinyusb_device
        tinyusb_board
        hardware_pio
)

pico_set_binary_type(JTAGprobe_smp_pio_std_weact copy_to_ram)

pico_add_extra_outputs(JTAGprobe_smp_pio_std_weact)
