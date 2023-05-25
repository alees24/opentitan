// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sw/device/lib/dif/dif_pinmux.h"
#include "sw/device/lib/runtime/ibex.h"
#include "sw/device/lib/runtime/log.h"
#include "sw/device/lib/runtime/print.h"
#include "sw/device/lib/testing/pinmux_testutils.h"
#include "sw/device/lib/testing/test_framework/check.h"
#include "sw/device/lib/testing/test_framework/ottf_main.h"
#include "sw/device/lib/testing/tinyusb/src/class/hid/hid_device.h"
#include "sw/device/lib/testing/tinyusb/src/tusb.h"
#include "sw/device/lib/testing/usbdev_tinyusb.h"

#include "hw/top_earlgrey/sw/autogen/top_earlgrey.h"  // Generated.

/**
 * USBDEV TinyUSB context
 */
// static
usbdev_tusb_ctx_t usbdev_tusb;
/**
 * Pinmux handle
 */
static dif_pinmux_t pinmux;

//#include "sw/device/lib/testing/tinyusb/hw/bsp/board.h"

uint32_t board_millis(void) {
  uint64_t now = ibex_mcycle_read();
  // TODO: this is just a crude approximation
  return (uint32_t)(now >> 13);
}

static const char signon[] =
    "Hello from OpenTitan!"
    "I am completely operational and all of my circuits are functioning "
    "perfectly!";

uint32_t board_button_read(void) {
  static int t = 0;
  if (++t >= 20)
    t = 0;
  return !t;
}

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// Interface index depends on the order in configuration descriptor
enum { ITF_KEYBOARD = 0, ITF_MOUSE = 1 };

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void hid_task(void);

/*------------- MAIN -------------*/
int main(void) {
  //  board_init();

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  while (1) {
    tud_task();  // tinyusb device task
    led_blinking_task();

    hid_task();
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) { blink_interval_ms = BLINK_MOUNTED; }

// Invoked when device is unmounted
void tud_umount_cb(void) { blink_interval_ms = BLINK_NOT_MOUNTED; }

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) { blink_interval_ms = BLINK_MOUNTED; }

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

void hid_task(void) {
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if (board_millis() - start_ms < interval_ms)
    return;  // not enough time
  start_ms += interval_ms;

  uint32_t const btn = board_button_read();

  // Remote wakeup
  if (tud_suspended() && btn) {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }

  /*------------- Keyboard -------------*/
  if (tud_hid_n_ready(ITF_KEYBOARD)) {
    // use to avoid send multiple consecutive zero report for keyboard
    static bool has_key = false;

    if (btn) {
      uint8_t keycode[6] = {0};
      //      keycode[0] = HID_KEY_A;

      static int sig_idx = 0;
      int key = signon[sig_idx];
      if (++sig_idx >= sizeof(signon))
        sig_idx = 0;

      if (key >= 'a' && key <= 'z')
        key = (key - 'a') + 'A';
      if (key >= 'A' && key <= 'Z') {
        key = key - 'A' + HID_KEY_A;
        keycode[0] = key;
        has_key = true;
      } else if (key == ' ') {
        keycode[0] = HID_KEY_SPACE;
        has_key = true;
      } else if (key == '\0') {
        keycode[0] = HID_KEY_ENTER;
        has_key = true;
      } else {
        has_key = false;
      }

      if (has_key) {
        tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, keycode);
      } else {
        // send empty key report if previously has key pressed
        tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, NULL);
      }
    } else {
      // send empty key report if previously has key pressed
      if (has_key) {
        tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, NULL);
      }
      has_key = false;
    }
  }

  /*------------- Mouse -------------*/
  if (tud_hid_n_ready(ITF_MOUSE)) {
    if (btn) {
      int8_t const delta = 5;

      // no button, right + down, no scroll pan
      tud_hid_n_mouse_report(ITF_MOUSE, 0, 0x00, delta, delta, 0, 0);
    }
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
  // TODO not Implemented
  (void)itf;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
  // TODO set LED based on CAPLOCK, NUMLOCK etc...
  (void)itf;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)bufsize;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms)
    return;  // not enough time
  start_ms += blink_interval_ms;

  //  board_led_write(led_state);
  led_state = 1 - led_state;  // toggle
}

OTTF_DEFINE_TEST_CONFIG();

bool test_main(void) {
  CHECK(kDeviceType == kDeviceSimVerilator || kDeviceType == kDeviceFpgaCw310,
        "This test is not expected to run on platforms other than the "
        "Verilator simulation or CW310 FPGA. It needs the USB DPI model "
        "or host application.");

  LOG_INFO("Running USBDEV MSC test");

  CHECK_DIF_OK(dif_pinmux_init(
      mmio_region_from_addr(TOP_EARLGREY_PINMUX_AON_BASE_ADDR), &pinmux));
  pinmux_testutils_init(&pinmux);
  CHECK_DIF_OK(dif_pinmux_input_select(
      &pinmux, kTopEarlgreyPinmuxPeripheralInUsbdevSense,
      kTopEarlgreyPinmuxInselIoc7));

  usbdev_tinyusb_init(&usbdev_tusb, false, false, false);

  tusb_init();

  tud_init(BOARD_TUD_RHPORT);

  LOG_INFO("Done init");

  while (true) {
    usbdev_tinyusb_poll(&usbdev_tusb);

    tud_task();
    led_blinking_task();

    hid_task();
  }

  return true;
}
