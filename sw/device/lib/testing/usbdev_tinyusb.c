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
static const bool irq_mode = true;
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

static void collect_buffer(usbdev_tusb_ctx_t *ctx, unsigned ep, uint8_t *buffer,
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

bool collect_buffers(usbdev_tusb_ctx_t *ctx, unsigned ep, uint8_t *buffer,
                     uint16_t total_length) {
  // Have we already got one or more pending buffers?
  if (ctx->ep[ep].recv.nbufs > 0U) {
    do {
      collect_buffer(ctx, ep, buffer, total_length, ctx->ep[ep].recv.bufs[0].is_setup, &ctx->ep[ep].recv.bufs[0].buf);

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

void rx_callback(void *ctx_v, dif_usbdev_rx_packet_info_t packet_info,
                 dif_usbdev_buffer_t buffer) {
  usbdev_tusb_ctx_t *ctx = (usbdev_tusb_ctx_t *)ctx_v;
  uint8_t ep = packet_info.endpoint;
  size_t bytes_read = 0U;

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
    CHECK(nbufs < 16U);  // TODO: recovery code, or report dropping at least?!

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

  if (!usb_testutils_buffer_send(&usbutils, ep, buffer, total_bytes, flags)) {
    ctx->ep[ep].sending = false;
    return false;
  }

  return true;
}

void tx_done_callback(void *ctx_v, usb_testutils_xfr_result_t result) {
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

void usbdev_tinyusb_poll(usbdev_tusb_ctx_t *ctx) {
  usb_testutils_poll(ctx->usbutils);
}

void usbdev_tinyusb_endpoint_setup(usbdev_tusb_ctx_t *ctx, uint8_t ep,
                                   unsigned dir,
                                   usb_testutils_out_transfer_mode_t out_mode) {
  if (dir == USBDEV_ENDPOINT_DIR_IN) {
    usb_testutils_in_endpoint_setup(ctx->usbutils, ep, &ctx->ep[ep],
                                    tx_done_callback, NULL, NULL);
  } else {
    usb_testutils_out_endpoint_setup(ctx->usbutils, ep, out_mode, ctx,
                                     rx_callback, NULL);
  }

  LOG_INFO("set up ep %u dir %u out mode %u\n", ep, dir, out_mode);
}

void usbdev_tinyusb_init(usbdev_tusb_ctx_t *ctx, bool pinflip,
                         bool en_diff_rcvr, bool tx_use_d_se0) {
  ctx->usbutils = &usbutils;
  // tx_done_callback handler requires the endpoint number
  for (unsigned ep = 0U; ep < USBDEV_NUM_ENDPOINTS; ep++) {
    ctx->ep[ep].ep = ep;
  }

  usb_testutils_init(ctx->usbutils, pinflip, en_diff_rcvr, tx_use_d_se0);

  // Set up Endpoint Zero for the Default Control Pipe
  usb_testutils_in_endpoint_setup(ctx->usbutils, 0, &ctx->ep[0],
                                  tx_done_callback, NULL, NULL);
  usb_testutils_out_endpoint_setup(ctx->usbutils, 0, kUsbdevOutMessage, ctx,
                                   rx_callback, NULL);

  // Enable reception of SETUP packets on Endpoint Zero
  CHECK_DIF_OK(dif_usbdev_endpoint_setup_enable(ctx->usbutils->dev, 0,
                                                kDifToggleEnabled));
}

void usbdev_tinyusb_connect(usbdev_tusb_ctx_t *ctx) {
  LOG_INFO("Enabling interface");
  CHECK_DIF_OK(
      dif_usbdev_interface_enable(ctx->usbutils->dev, kDifToggleEnabled));
}

void usbdev_tinyusb_disconnect(usbdev_tusb_ctx_t *ctx) {
  LOG_INFO("Disabling interface");
  CHECK_DIF_OK(
      dif_usbdev_interface_enable(ctx->usbutils->dev, kDifToggleDisabled));
}

void usbdev_tinyusb_set_address(usbdev_tusb_ctx_t *ctx, uint8_t dev_address) {
  LOG_INFO("Setting address %u", dev_address);
  CHECK_DIF_OK(dif_usbdev_address_set(ctx->usbutils->dev, dev_address));
}

void usbdev_tinyusb_stall(usbdev_tusb_ctx_t *ctx, uint8_t ep, int dir,
                          bool enable) {
  dif_usbdev_endpoint_id_t endpoint = {
      .number = ep,
      .direction = dir,
  };

  if (verbose) {
    LOG_INFO("ep %u dir %u %sabling STALL", ep, dir, enable ? "En" : "Dis");
  }

  if (enable) {
    CHECK_DIF_OK(dif_usbdev_endpoint_stall_enable(ctx->usbutils->dev, endpoint,
                                                  kDifToggleEnabled));
  } else {
    CHECK_DIF_OK(dif_usbdev_endpoint_stall_enable(ctx->usbutils->dev, endpoint,
                                                  kDifToggleDisabled));
  }
}
