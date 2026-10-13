cmake_minimum_required(VERSION 3.20)

include(pico_sdk_import.cmake)

project(JTAGprobe_smp_pio_std C CXX ASM)

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

add_executable(JTAGprobe_smp_pio_std

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
)

pico_generate_pio_header(JTAGprobe_smp_pio_std ${CMAKE_CURRENT_LIST_DIR}/src/jtag.pio)

target_include_directories(JTAGprobe_smp_pio_std PRIVATE

        CMSIS_6/CMSIS/Core/Include/
        include/
        )

target_compile_options(JTAGprobe_smp_pio_std PRIVATE -Wall)

# src/ also holds cmsis-plus/os-app-config.h, the µOS++ configuration.
target_include_directories(JTAGprobe_smp_pio_std PRIVATE src)

target_compile_definitions (JTAGprobe_smp_pio_std PRIVATE
	PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1
	OS_USE_OS_APP_CONFIG_H
	# The kernel's malloc() already runs under the scheduler lock, across
	# both cores; the SDK's wrapper must not add a Pico mutex on top of it.
	PICO_USE_MALLOC_MUTEX=0
)


target_link_libraries(JTAGprobe_smp_pio_std PRIVATE
        micro-os-plus::rp2040
        pico_stdlib
        pico_unique_id
        tinyusb_device
        tinyusb_board
        hardware_pio
)

pico_set_binary_type(JTAGprobe_smp_pio_std copy_to_ram)

pico_add_extra_outputs(JTAGprobe_smp_pio_std)
