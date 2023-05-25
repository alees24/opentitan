// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef OPENTITAN_SW_DEVICE_LIB_TESTING_USBDEV_TINYUSB_H_
#define OPENTITAN_SW_DEVICE_LIB_TESTING_USBDEV_TINYUSB_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sw/device/lib/dif/dif_usbdev.h"
#include "sw/device/lib/runtime/log.h"
#include "sw/device/lib/testing/test_framework/check.h"
#include "sw/device/lib/testing/tinyusb/src/tusb.h"
#include "sw/device/lib/testing/usb_testutils.h"

#define USBDEV_TUSB_SEND_BUFS 8U
#define USBDEV_TUSB_RECV_BUFS 8U

typedef struct usbdev_tusb_ctx usbdev_tusb_ctx_t;

/**
 * Buffers that we have received but not yet passed to TinyUSB
 */
typedef struct usbdev_tusb_rx_buf {
  /**
   * Received buffer contains a SETUP packet?
   */
  bool is_setup;
  /**
   * Description of received buffer
   */
  dif_usbdev_buffer_t buf;
} usbdev_tusb_rx_buf_t;

/**
 * State information for a single endpoint pair
 */
typedef struct usbdev_tusb_ep_ctx {
  /**
   * Endpoint number
   */
  uint8_t ep;
  /**
   * Transmission to host in progress?
   */
  bool sending;
  /**
   * Length of data being sent, in bytes
   */
  uint16_t send_len;
  /**
   * Receive path requires a queue of buffers because we cannot return data to
   * the TinyUSB stack immediately; we must wait until TinyUSB provides a buffer
   * for the received packet.
   */
  struct {
    /**
     * TinyUSB-supplied buffer for data reception
     */
    uint8_t *buffer;
    /**
     * Size in bytes of reception buffer
     */
    uint16_t len;
    /**
     * Has reception buffer been supplied?
     */
    bool supplied;
    /**
     * Number of received buffers collected
     */
    unsigned nbufs;
    /**
     * Buffers that we have received but not yet passed to TinyUSB
     */
    usbdev_tusb_rx_buf_t bufs[USBDEV_TUSB_RECV_BUFS];
  } recv;
} usbdev_tusb_ep_ctx_t;

/**
 * State information for TinyUSB on usb_testutils
 */
struct usbdev_tusb_ctx {
  usb_testutils_ctx_t *usbutils;
  /**
   * State information for each pair of endpoints
   */
  usbdev_tusb_ep_ctx_t ep[USBDEV_NUM_ENDPOINTS];
};

/**
 * Call regularly to poll the usbdev interface
 *
 * @param ctx usbdev tinyusb context pointer
 */
inline status_t usbdev_tinyusb_poll(usbdev_tusb_ctx_t *ctx) {
  return usb_testutils_poll(ctx->usbutils);
}

/**
 * Initialize the usbdev tinyusb interface
 *
 * @param ctx uninitialized usbdev tinyusb context pointer
 * @param pinflip boolean to indicate if PHY should be configured for D+/D- flip
 * @param en_diff_rcvr boolean to indicate if PHY should enable an external
 *                     differential receiver, activating the single-ended D
 *                     input
 * @param tx_use_d_se0 boolean to indicate if PHY uses D/SE0 for TX instead of
 *                     Dp/Dn
 * @return The result of the operation
 */
//OT_WARN_UNUSED_RESULT
status_t usbdev_tinyusb_init(usbdev_tusb_ctx_t *ctx, bool pinflip,
                             bool en_diff_rcvr, bool tx_use_d_se0);

status_t usbdev_tinyusb_in_endpoint_setup(usbdev_tusb_ctx_t *ctx, uint8_t ep, usb_testutils_transfer_type_t ep_type);

status_t usbdev_tinyusb_out_endpoint_setup(
    usbdev_tusb_ctx_t *ctx, uint8_t ep, usb_testutils_transfer_type_t ep_type,
    usb_testutils_out_transfer_mode_t out_mode);

bool usbdev_tinyusb_send(usbdev_tusb_ctx_t *ctx, uint8_t ep,
                         const uint8_t *buffer, uint16_t total_bytes);

bool usbdev_tinyusb_recv(usbdev_tusb_ctx_t *ctx, uint8_t ep, uint8_t *buffer,
                         uint16_t total_bytes);

inline void usbdev_tinyusb_connect(usbdev_tusb_ctx_t *ctx) {
  CHECK_DIF_OK(
      dif_usbdev_interface_enable(ctx->usbutils->dev, kDifToggleEnabled));
}

inline void usbdev_tinyusb_disconnect(usbdev_tusb_ctx_t *ctx) {
  CHECK_DIF_OK(
      dif_usbdev_interface_enable(ctx->usbutils->dev, kDifToggleDisabled));
}

inline void usbdev_tinyusb_set_address(usbdev_tusb_ctx_t *ctx,
                                       uint8_t dev_address) {
  CHECK_DIF_OK(dif_usbdev_address_set(ctx->usbutils->dev, dev_address));
}

inline void usbdev_tinyusb_stall(usbdev_tusb_ctx_t *ctx, uint8_t ep, unsigned dir,
                                 bool enable) {
  dif_usbdev_endpoint_id_t endpoint = {
      .number = ep,
      .direction = dir,
  };

  CHECK_DIF_OK(dif_usbdev_endpoint_stall_enable(ctx->usbutils->dev, endpoint, enable ? kDifToggleEnabled : kDifToggleDisabled));
}

#endif  // OPENTITAN_SW_DEVICE_LIB_TESTING_USBDEV_TINYUSB_H_
