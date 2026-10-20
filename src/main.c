/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2021 Peter Lawrence
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <cmsis-plus/rtos/os-c-api.h>
#include <micro-os-plus-rp2040.h>

#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"

#include "get_serial.h"
#include "tusb_edpt_handler.h"
#include "DAP.h"
#include "board_pico_config.h"
#include "DAP_config.h"
#include "cdc_uart.h"
#include "probe_os.h"
#include "led.h"



// Define JTAG buffer size
#define JTAG_BUFFER_SIZE 64

// Define JTAG_BUILD_REQUEST and required variables



#define JTAG_BUILD_REQUEST(operation, register) (((operation) << 16) | (register))


// UART0 for debugprobe debug
// UART1 for debugprobe to target device

static uint8_t TxDataBuffer[DAP_PACKET_SIZE];
static uint8_t RxDataBuffer[DAP_PACKET_SIZE];
//static uint8_t JTAGBuffer[JTAG_BUFFER_SIZE];




// µOS++ thread definitions, priorities and placement (see probe_os.h).
//
// Core 0 serves the host: tinyusb ("usb") and the UART bridge ("uart"; it
// must preempt USB, as characters get lost otherwise). Core 1 does nothing
// but run DAP commands ("dap"), so the JTAG/SWD bit-banging -- and with it
// the target's semihosting traffic -- overlaps the USB transfers of the
// previous and next packets instead of taking turns with them.

#define THREADED 1

#define UART_TASK_PRIO os_thread_priority_high
#define TUD_TASK_PRIO  os_thread_priority_above_normal
#define DAP_TASK_PRIO  os_thread_priority_normal

#define CORE0 (1u << 0)
#define CORE1 (1u << 1)

#define THREAD_STACK_SIZE_BYTES 2048

static os_thread_t uart_thread_obj, tud_thread_obj, dap_thread_obj;

static os_thread_stack_allocation_element_t
    uart_stack[THREAD_STACK_SIZE_BYTES / sizeof (os_thread_stack_allocation_element_t)],
    tud_stack[THREAD_STACK_SIZE_BYTES / sizeof (os_thread_stack_allocation_element_t)],
    dap_stack[THREAD_STACK_SIZE_BYTES / sizeof (os_thread_stack_allocation_element_t)];

// The objects the threads share (probe_os.h).
os_mutex_t usb_mutex;
os_semaphore_t usb_sem;
os_semaphore_t dap_sem;
os_semaphore_t uart_sem;




//USB thread to be written here

void *usb_thread(void *ptr)
{
    do {
        // Every tinyusb call is made with usb_mutex held.
        os_mutex_lock(&usb_mutex);
        tud_task();

        // Hand the DAP thread's responses to the IN endpoint, re-arm OUT.
        dap_edpt_service();

        // While DAP packets are flowing, keep polling instead of sleeping:
        // the next OUT packet is then picked up without a semaphore wake-up
        // and a context switch. "uart" still preempts this thread.
        bool more = tud_task_event_ready() || dap_edpt_active();
        os_mutex_unlock(&usb_mutex);

        // Go to sleep for up to a tick if nothing to do; the USB IRQ and
        // the DAP thread wake it at once.
        if (!more)
            os_semaphore_timed_wait(&usb_sem, 1);
    } while (1);
    return NULL;
}

// tinyusb queued an event (USB IRQ, core 0): wake the "usb" thread.
void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr)
{
    (void) rhport;
    (void) eventid;
    (void) in_isr;
    os_semaphore_post(&usb_sem);
}

// tinyusb runs without an OS (OPT_OS_NONE); its millisecond clock.
uint32_t tusb_time_millis_api(void)
{
    return to_ms_since_boot(get_absolute_time());
}




// Workaround API change in 0.13
#if (TUSB_VERSION_MAJOR == 0) && (TUSB_VERSION_MINOR <= 12)
#define tud_vendor_flush(x) ((void)0)
#endif

//test code for code execution





static void create_thread(os_thread_t *th, const char *name, os_thread_func_t func,
                          void *stack, size_t stack_size, os_thread_prio_t prio,
                          uint32_t affinity)
{
    os_thread_attr_t attr;
    os_thread_attr_init(&attr);
    attr.th_stack_address = stack;
    attr.th_stack_size_bytes = stack_size;
    attr.th_priority = prio;
    attr.th_cpu_affinity = affinity;
    os_thread_create(th, name, func, NULL, &attr);
}

//Main function begins here ,Call all the required functions here to be executed JTAG not yet declared
//
// The Pico SDK boots the board and calls the µOS++ kernel's main(), which
// starts the scheduler on core 0 and runs os_main() in the "main" thread,
// pinned to core 0.

int os_main(int argc, char *argv[]) {

    (void) argc;
    (void) argv;

    probe_info("Welcome to jtagprobe!\n");
    DAP_Setup();
    board_init();
    // Status LED: on, blinking while TCK/SWCLK is in use (led.c).
    led_init();
    usb_serial_init();

    os_mutex_create(&usb_mutex, "usb", NULL);
    os_semaphore_binary_create(&usb_sem, "usb", 0);
    os_semaphore_binary_create(&dap_sem, "dap", 0);
    os_semaphore_binary_create(&uart_sem, "uart", 0);

    tusb_init();
    cdc_uart_init();




    //led_init removed for the sake of simiplicity
    //led_init();

    if (THREADED) {
        /* UART needs to preempt USB as if we don't, characters get lost */
        create_thread(&uart_thread_obj, "UART", cdc_thread, uart_stack,
                      sizeof(uart_stack), UART_TASK_PRIO, CORE0);
        create_thread(&tud_thread_obj, "TUD", usb_thread, tud_stack,
                      sizeof(tud_stack), TUD_TASK_PRIO, CORE0);
        /* The debug thread gets core 1 to itself */
        create_thread(&dap_thread_obj, "DAP", dap_thread, dap_stack,
                      sizeof(dap_stack), DAP_TASK_PRIO, CORE1);

        /* Core 1 joins the scheduler, and picks up the DAP thread. */
        os_rp2040_start_core1();

        /* The threads never end. */
        os_thread_join(&tud_thread_obj, NULL);
    }

    while (!THREADED) {
        tud_task();





        // In the non-threaded mode, continuously perform JTAG operations and forward data to USB
        // Example: JTAG_Transfer(request, &data);
        // Example: tud_hid_report(0, data, JTAG_BUFFER_SIZE);
        // Perform JTAG operation to read the IDCODE register



        uint32_t idcode;
        uint32_t request;

        request = JTAG_BUILD_REQUEST(ID_DAP_Transfer | DAP_TRANSFER_RnW | DAP_TRANSFER_APnDP, ID_DAP_JTAG_IDCODE);
        JTAG_Transfer(request, &idcode);

        // Forward the IDCODE data to the USB HID interface
        tud_hid_report(0, (uint8_t*)&idcode, sizeof(uint32_t));

        //Report buffer size
        //tud_hid_report(0, data, JTAG_BUFFER_SIZE);

        // Delay for a short period to avoid busy-waiting
        os_sysclock_sleep_for(10);















#if (PROBE_DEBUG_PROTOCOL == PROTO_DAP_V2)
      if (tud_vendor_available()) {
          uint32_t resp_len;
          tud_vendor_read(RxDataBuffer, sizeof(RxDataBuffer));
          resp_len = DAP_ProcessCommand(RxDataBuffer, TxDataBuffer);
          tud_vendor_write(TxDataBuffer, resp_len);
      }
#endif
    }

    return 0;
}

//Main function ends  here















uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  /*TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;
*/
  return 0;
}
















void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* RxDataBuffer, uint16_t bufsize)
{
  uint32_t response_size = TU_MIN(CFG_TUD_HID_EP_BUFSIZE, bufsize);

  // This doesn't use multiple report and report ID
  (void) itf;
  (void) report_id;
  (void) report_type;

  // You can process the received HID report from the host here if needed
  //USB to JTAG to be processed here
  // For now, let's forward it to JTAG (DAP) thread
  // Example:  xQueueSendToBack(JTAGQueue, RxDataBuffer, 0);

    DAP_ProcessCommand(RxDataBuffer, TxDataBuffer);
    tud_hid_report(0, TxDataBuffer, response_size);
}













//Required only to be compatible with ms windows devices





#if (PROBE_DEBUG_PROTOCOL == PROTO_DAP_V2)
extern uint8_t const desc_ms_os_20[];

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  switch (request->bmRequestType_bit.type)
  {
    case TUSB_REQ_TYPE_VENDOR:
      switch (request->bRequest)
      {
        case 1:
          if ( request->wIndex == 7 )
          {
            // Get Microsoft OS 2.0 compatible descriptor
            uint16_t total_len;
            memcpy(&total_len, desc_ms_os_20+8, 2);

            return tud_control_xfer(rhport, request, (void*) desc_ms_os_20, total_len);
          }else
          {
            return false;
          }

        default: break;
      }
    break;
    default: break;
  }

  // stall unknown request
  return false;
}
#endif
