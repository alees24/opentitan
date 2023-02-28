/*
 * The MIT License (MIT)
 *
 * Copyright 2019 Sony Semiconductor Solutions Corporation
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
 * Device driver for OpenTitan EarlGrey USB device.
 */

#include "sw/device/lib/dif/dif_pinmux.h"
#include "sw/device/lib/testing/pinmux_testutils.h"
#include "sw/device/lib/testing/usbdev_tinyusb.h"

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_OTEARLGREY

#include <errno.h>

#include "../../../device/dcd.h"
#include "../../../device/usbd_pvt.h"

static const bool verbose = false;

/**
 * Pinmux handle
 */
static dif_pinmux_t pinmux;

//  Single instance of TinyUSB<->usb_testutils bridge
static usbdev_tusb_ctx_t usbdev_tusb;

// TODO: what does rhport denote? Can we use this to locate context information?
void dcd_init(uint8_t rhport) {
  (void)rhport;

  CHECK_DIF_OK(dif_pinmux_init(
      mmio_region_from_addr(TOP_EARLGREY_PINMUX_AON_BASE_ADDR), &pinmux));
  pinmux_testutils_init(&pinmux);
  CHECK_DIF_OK(dif_pinmux_input_select(
      &pinmux, kTopEarlgreyPinmuxPeripheralInUsbdevSense,
      kTopEarlgreyPinmuxInselIoc7));

  usbdev_tinyusb_init(&usbdev_tusb, false, false, false);

  usbdev_tinyusb_connect(&usbdev_tusb);
}

// Polling routine still required at present because we're not using interrupts
void dcd_service(void) {
  usbdev_tinyusb_poll(&usbdev_tusb);
}

// Enable device interrupt
void dcd_int_enable(uint8_t rhport) { (void)rhport; }

// Disable device interrupt
void dcd_int_disable(uint8_t rhport) { (void)rhport; }

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
  (void)rhport;
  (void)dev_addr;

  // Response with zlp status
  dcd_edpt_xfer(rhport, 0x80, NULL, 0);

  usbdev_tinyusb_set_address(&usbdev_tusb, dev_addr);
}

void dcd_remote_wakeup(uint8_t rhport) { (void)rhport; }

void dcd_connect(uint8_t rhport) {
  (void)rhport;

  usbdev_tinyusb_connect(&usbdev_tusb);
}

void dcd_disconnect(uint8_t rhport) {
  (void)rhport;

  usbdev_tinyusb_disconnect(&usbdev_tusb);
}

void dcd_sof_enable(uint8_t rhport, bool en) {
  (void)rhport;
  (void)en;

  // TODO implement later
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

bool dcd_edpt_open(uint8_t rhport,
                   tusb_desc_endpoint_t const *p_endpoint_desc) {
  uint8_t epnum = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
  uint8_t const dir = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);
  uint8_t xfrtype = 0;
  uint16_t const ep_mps = tu_edpt_packet_size(p_endpoint_desc);

  LOG_INFO("open ep %u dir %u xfrtype %u mps %u\n", epnum, dir, xfrtype,
           ep_mps);

  usbdev_tusb_ctx_t *ctx = &usbdev_tusb;

  unsigned dif_dir = (dir == TUSB_DIR_IN);

  usb_testutils_out_transfer_mode_t out_mode = kUsbdevOutMessage;

  switch (p_endpoint_desc->bmAttributes.xfer) {
    case 1:
      out_mode = kUsbdevOutStream;
      break;
    case 2:
      out_mode = kUsbdevOutStream;
      break;
    case 3:
      out_mode = kUsbdevOutStream;
      break;
    default:
      break;
  }

  usbdev_tinyusb_endpoint_setup(ctx, epnum, dif_dir, out_mode);

  return true;
}

void dcd_edpt_close_all(uint8_t rhport) {
  (void)rhport;
  // TODO implement dcd_edpt_close_all()
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer,
                   uint16_t total_bytes) {
  (void)rhport;

  uint8_t epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  usbdev_tusb_ctx_t *ctx = &usbdev_tusb;

  //  LOG_INFO("edpt_xfer 0x%x\n", ep_addr);

  if (dir == TUSB_DIR_IN) {
    usbdev_tinyusb_send(ctx, epnum, buffer, total_bytes);
  } else {
    usbdev_tinyusb_recv(ctx, epnum, buffer, total_bytes);
  }

  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  (void)rhport;

  uint8_t epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  usbdev_tusb_ctx_t *ctx = &usbdev_tusb;

  if (verbose) {
    LOG_INFO("edpt_stall %u\n", ep_addr);
  }
  usbdev_tinyusb_stall(ctx, epnum, dir, true);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  (void)rhport;

  uint8_t epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  usbdev_tusb_ctx_t *ctx = &usbdev_tusb;

  if (verbose) {
    LOG_INFO("edpt_clear_stall %u\n", ep_addr);
  }
  usbdev_tinyusb_stall(ctx, epnum, dir, false);
}

#endif
