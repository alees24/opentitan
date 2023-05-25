// SPDX-License-Identifier: Apache-2.0

#include "sw/device/lib/testing/usbdev_tinyusb.h"

#include "sw/device/lib/dif/dif_usbdev.h"
#include "sw/device/lib/runtime/log.h"
#include "sw/device/lib/runtime/print.h"
#include "sw/device/lib/testing/test_framework/check.h"
#include "sw/device/lib/testing/tinyusb/src/device/dcd.h"
#include "sw/device/lib/testing/tinyusb/src/device/usbd_pvt.h"
#include "sw/device/lib/testing/tinyusb/src/tusb.h"

#include "hw/top_earlgrey/sw/autogen/top_earlgrey.h"

static usb_testutils_ctx_t usbutils;
/**
 * Calling TinyUSB (as if) from background IRQ code?
 */
static const bool irq_mode = false;
/**
 * Verbose reporting?
 */
static const bool verbose = false;

// Dump a sequence of bytes as hexadecimal and ASCII for diagnostic purposes
static void buffer_dump(const uint8_t *data, size_t n) {
  base_hexdump_fmt_t fmt = {
      .bytes_per_word = 1,
      .words_per_line = 0x20u,
      .alphabet = &kBaseHexdumpDefaultFmtAlphabet,
  };

  base_hexdump_with(fmt, (char *)data, n);
}

static void collect_buffer(usbdev_tusb_ctx_t *ctx, uint8_t ep, uint8_t *buffer,
                           uint16_t total_length, bool is_setup,
                           dif_usbdev_buffer_t *buf) {
  size_t bytes_read = 0U;

  if (!ep) {
    // Endpoint zero is set to block further OUT packets;
    // specifically this is message pipe behavior rather than just Endpoint Zero
    CHECK_DIF_OK(dif_usbdev_endpoint_out_enable(
        ctx->usbutils->dev, /*endpoint=*/0, kDifToggleEnabled));
  }

  if (is_setup) {
    static uint8_t data[8];

    CHECK_DIF_OK(dif_usbdev_buffer_read(ctx->usbutils->dev,
                                        usbutils.buffer_pool, buf, data,
                                        buf->remaining_bytes, &bytes_read));
    CHECK(bytes_read == 8U);

    if (verbose) {
      LOG_INFO("ep %u rx SETUP", ep);
      buffer_dump(data, bytes_read);
    }
    dcd_event_setup_received(0, data, irq_mode);
  } else {
    size_t len = buf->remaining_bytes;
    size_t bytes_read = 0U;

    if (len) {
      CHECK(buffer);
      CHECK_DIF_OK(dif_usbdev_buffer_read(ctx->usbutils->dev,
                                          usbutils.buffer_pool, buf, buffer,
                                          len, &bytes_read));
    } else {
      // A Zero Length Packet carries no data and just completes the transfer,
      // but the buffer must still be returned for reuse
      CHECK_DIF_OK(dif_usbdev_buffer_return(ctx->usbutils->dev,
                                            usbutils.buffer_pool, buf));
    }

    if (verbose) {
      LOG_INFO("ep %u rx DATA len %u", ep, bytes_read);
      if (bytes_read) {
        buffer_dump(buffer, bytes_read);
      }
    }
    dcd_event_xfer_complete(0, ep, bytes_read, XFER_RESULT_SUCCESS, irq_mode);
  }
}

static bool collect_buffers(usbdev_tusb_ctx_t *ctx, uint8_t ep,
                            uint8_t *buffer, uint16_t total_length) {
  // Have we already got one or more pending buffers?
  if (ctx->ep[ep].recv.nbufs > 0U) {
    do {
      collect_buffer(ctx, ep, buffer, total_length,
                     ctx->ep[ep].recv.bufs[0].is_setup,
                     &ctx->ep[ep].recv.bufs[0].buf);

      // Shuffle the remaining buffers
      for (unsigned idx = 1U; idx < ctx->ep[ep].recv.nbufs; idx++) {
        ctx->ep[ep].recv.bufs[idx - 1U] = ctx->ep[ep].recv.bufs[idx];
      }
      ctx->ep[ep].recv.nbufs--;

      // Proceed to process any SETUP packets now that the DATA packet is out of
      // the way...
    } while (ctx->ep[ep].recv.nbufs > 0U && ctx->ep[ep].recv.bufs[0].is_setup);

    return true;
  }
  return false;
}

static status_t rx_callback(void *ctx_v, dif_usbdev_rx_packet_info_t packet_info,
                        dif_usbdev_buffer_t buffer) {
  usbdev_tusb_ctx_t *ctx = (usbdev_tusb_ctx_t *)ctx_v;
  uint8_t ep = packet_info.endpoint;

  if (verbose) {
    LOG_INFO("rx ep %u setup %c len %u", ep, packet_info.is_setup ? 'Y' : 'N',
             packet_info.length);
  }

  unsigned nbufs = ctx->ep[ep].recv.nbufs;

  // If buffer(s) are already queued, then this one must not jump the queue.
  // This packet can be collected immediately if (i) it's a SETUP packet or
  // (ii) a receive buffer has already been supplied
  if (nbufs > 0U || (!packet_info.is_setup && !ctx->ep[ep].recv.supplied)) {
    // Collect the buffer and handle it later
    if (verbose) {
      LOG_INFO("ep %u queuing at index %u", ep, nbufs);
    }
    // TODO: recovery code, or report dropping at least?!
    TRY_CHECK(nbufs < USBDEV_TUSB_RECV_BUFS);

    ctx->ep[ep].recv.bufs[nbufs].is_setup = packet_info.is_setup;
    ctx->ep[ep].recv.bufs[nbufs].buf = buffer;

    ctx->ep[ep].recv.nbufs = nbufs + 1U;
  } else {
    collect_buffer(ctx, ep, ctx->ep[ep].recv.buffer, ctx->ep[ep].recv.len,
                   packet_info.is_setup, &buffer);
    if (!packet_info.is_setup) {
      ctx->ep[ep].recv.supplied = false;
    }
  }

  return OK_STATUS();
}

bool usbdev_tinyusb_send(usbdev_tusb_ctx_t *ctx, uint8_t ep,
                             const uint8_t *buffer, uint16_t total_bytes) {
  // TinyUSB sends its own ZLP so we don't need usb_testutils to handle that
  usb_testutils_xfr_flags_t flags = kUsbTestutilsXfrDoubleBuffered;

  if (verbose) {
    LOG_INFO("ep %u sending %u byte(s)", ep, total_bytes);
    buffer_dump(buffer, total_bytes);
  }

  ctx->ep[ep].sending = true;
  ctx->ep[ep].send_len = total_bytes;

  status_t status = usb_testutils_transfer_send(&usbutils, ep, buffer, total_bytes, flags);
  if (!status_ok(status)) {
    ctx->ep[ep].sending = false;
    return false;
  }

  return true;
}

static status_t tx_done_callback(void *ctx_v, usb_testutils_xfr_result_t result) {
  usbdev_tusb_ep_ctx_t *ep_ctx = (usbdev_tusb_ep_ctx_t *)ctx_v;
  if (verbose) {
    LOG_INFO("tx_done callback on ep %u sending %c\n", ep_ctx->ep,
             ep_ctx->sending ? 'Y' : 'N');
  }

  if (ep_ctx->sending) {
    ep_ctx->sending = false;
    dcd_event_xfer_complete(0, ep_ctx->ep | 0x80, ep_ctx->send_len,
                            XFER_RESULT_SUCCESS, irq_mode);
  } else {
    LOG_INFO("ep %u Unexpected tx_done_callback", ep_ctx->ep);
  }
  return OK_STATUS();
}

/*
static void reset(void *ctx_v) {
  usbdev_tusb_ctx_t *ctx = (usbdev_tusb_ctx_t *)ctx_v;

  //  LOG_INFO("USB: Bus reset");

  for (uint8_t ep = 0U; ep < USBDEV_NUM_ENDPOINTS; ep++) {
    // Reset sending state
    ctx->ep[ep].sending = false;

    // Reset receiving state
    ctx->ep[ep].recv.nbufs = 0U;
    ctx->ep[ep].recv.supplied = false;
  }

  dcd_event_bus_signal(0, DCD_EVENT_BUS_RESET, irq_mode);
}
*/

static status_t link_callback(void *ctx_v, dif_usbdev_irq_state_snapshot_t snapshot,
                              dif_usbdev_link_state_t link_state) {
  return OK_STATUS();
}

bool usbdev_tinyusb_recv(usbdev_tusb_ctx_t *ctx, uint8_t ep, uint8_t *buffer,
                         uint16_t total_bytes) {
  if (verbose) {
    LOG_INFO("ep %u recv buffer %p total %u\n", ep, buffer, total_bytes);
  }

  if (collect_buffers(ctx, ep, buffer, total_bytes)) {
    return true;
  }

  // Retain details of this buffer until we receive the DATA packet
  if (verbose) {
    LOG_INFO("ep %u retaining buffer %p %u", ep, buffer, total_bytes);
  }
  ctx->ep[ep].recv.buffer = (uint8_t *)buffer;
  ctx->ep[ep].recv.len = total_bytes;
  ctx->ep[ep].recv.supplied = true;

  return true;
}


status_t usbdev_tinyusb_in_endpoint_setup(usbdev_tusb_ctx_t *ctx, uint8_t ep, usb_testutils_transfer_type_t ep_type) {
  if (verbose) {
    LOG_INFO("set up IN ep %u\n", ep);
  }
  return usb_testutils_in_endpoint_setup(ctx->usbutils, ep, ep_type, &ctx->ep[ep], tx_done_callback, NULL, NULL);
}

status_t usbdev_tinyusb_out_endpoint_setup(
    usbdev_tusb_ctx_t *ctx, uint8_t ep, usb_testutils_transfer_type_t ep_type,
    usb_testutils_out_transfer_mode_t out_mode) {
  if (verbose) {
    LOG_INFO("set up OUT ep %u out mode %u\n", ep, out_mode);
  }
  return usb_testutils_out_endpoint_setup(ctx->usbutils, ep, ep_type, out_mode, ctx, rx_callback, NULL);
}

status_t usbdev_tinyusb_init(usbdev_tusb_ctx_t *ctx, bool pinflip,
                             bool en_diff_rcvr, bool tx_use_d_se0) {
  ctx->usbutils = &usbutils;

  for (uint8_t ep = 0U; ep < USBDEV_NUM_ENDPOINTS; ep++) {
    // tx_done_callback handler requires the endpoint number
    ctx->ep[ep].ep = ep;

    // Reset sending state
    ctx->ep[ep].sending = false;

    // Reset receiving state
    ctx->ep[ep].recv.nbufs = 0U;
    ctx->ep[ep].recv.supplied = false;
  }

  TRY(usb_testutils_init(ctx->usbutils, pinflip, en_diff_rcvr, tx_use_d_se0));

  // Register our interest in link events
  TRY(usb_testutils_link_callback_register(ctx->usbutils, link_callback, ctx));

  // Set up Endpoint Zero for the Default Control Pipe
  TRY(usb_testutils_in_endpoint_setup(ctx->usbutils, 0, kUsbTransferTypeControl, &ctx->ep[0], tx_done_callback, NULL, NULL));
  // TODO:
  // reset);
  TRY(usb_testutils_out_endpoint_setup(ctx->usbutils, 0, kUsbTransferTypeControl, kUsbdevOutMessage, ctx, rx_callback, NULL));

  // Enable reception of SETUP packets on Endpoint Zero
  TRY(dif_usbdev_endpoint_setup_enable(ctx->usbutils->dev, 0,
                                                kDifToggleEnabled));
  return OK_STATUS();
}

// 'extern' declarations to give the inline functions in the corresponding
// header a link location.
extern void usbdev_tinyusb_connect(usbdev_tusb_ctx_t *ctx);
extern void usbdev_tinyusb_disconnect(usbdev_tusb_ctx_t *ctx);
extern status_t usbdev_tinyusb_poll(usbdev_tusb_ctx_t *ctx);
extern void usbdev_tinyusb_set_address(usbdev_tusb_ctx_t *ctx,
                                       uint8_t dev_address);
extern void usbdev_tinyusb_stall(usbdev_tusb_ctx_t *ctx, uint8_t ep, unsigned dir,
                                 bool enable);
