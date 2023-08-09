// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "usbdev_ctl.h"

// TODO: required for usleep presently
#include <unistd.h>

#include <cassert>
#include <cstdio>
#include <cstring>

// Dump a sequence of bytes as hexadecimal and ASCII for diagnostic purposes
static void buffer_dump(FILE *out, const uint8_t *data, size_t n) {
  static const char hex_digits[] = "0123456789abcdef";
  const unsigned ncols = 0x20u;
  char buf[ncols * 4u + 2u];

  while (n > 0u) {
    const unsigned chunk = (n > ncols) ? ncols : (unsigned)n;
    const uint8_t *row = data;
    unsigned idx = 0u;
    char *dp = buf;

    // Columns of hexadecimal bytes
    while (idx < chunk) {
      dp[0] = hex_digits[row[idx] >> 4];
      dp[1] = hex_digits[row[idx++] & 0xfu];
      dp[2] = ' ';
      dp += 3;
    }
    while (idx++ < ncols) {
      dp[2] = dp[1] = dp[0] = ' ';
      dp += 3;
    }

    // Printable ASCII characters
    for (idx = 0u; idx < chunk; idx++) {
      uint8_t ch = row[idx];
      *dp++ = (ch < ' ' || ch >= 0x80u) ? '.' : ch;
    }
    *dp = '\0';
    fprintf(out, "%s\n", buf);
    data += chunk;
    n -= chunk;
  }

  fflush(stdout);
}

// Stub callback function supplied to libusb.
void LIBUSB_CALL USBDevCtl::CbStubIN(struct libusb_transfer *xfr) {
  USBDevCtl *self = reinterpret_cast<USBDevCtl *>(xfr->user_data);
  self->CallbackIN(xfr);
}

void LIBUSB_CALL USBDevCtl::CbStubOUT(struct libusb_transfer *xfr) {
  USBDevCtl *self = reinterpret_cast<USBDevCtl *>(xfr->user_data);
  self->CallbackOUT(xfr);
}

bool USBDevCtl::Open(unsigned interface) {
  int rc = dev_->ClaimInterface(interface);
  if (rc < 0) {
    return dev_->ErrorUSB("ERROR: Claiming interface", rc);
  }

  // Retain the interface number
  interface_ = interface;

  // Remember the (assumed) endpoints which we're using
  epOut_ = interface + 1U;
  epIn_ = 0x80U | epOut_;

  // No transfers in progress
  xfr_ = nullptr;
  activeState_ = CtlIdle;

  // TODO: we should perhaps read this from the device somehow?
  maxPacketSize_ = USBDevice::kDevDataMaxPacketSize;

  return true;
}

// TODO: Perhaps we call these Stop()/Pause() and Continue();
void USBDevCtl::Close(bool wait) {
  SetClosing(true);

  if (wait) {
    if (verbose_) {
      std::cout << PrefixID() << "waiting to close" << std::endl;
    }
    while (activeState_ != CtlIdle) {
      dev_->Service();
static int cnt = 0;
if (++cnt >= 10000) {
 std::cout << PrefixID() << "xfr " << (void*)xfr_ << " act " << activeState_ << std::endl;
 cnt = 0;
} else {
 usleep(100);
}
    }
    if (verbose_) {
      std::cout << PrefixID() <<" closed" << std::endl;
    }
  }

  int rc = dev_->ReleaseInterface(interface_);
  if (rc < 0) {
    std::cerr << "" << std::endl;
  }
}

bool USBDevCtl::Resume() {
  SetClosing(false);

  int rc = dev_->ClaimInterface(interface_);
  if (rc < 0) {
    return dev_->ErrorUSB("ERROR: Claiming interface", rc);
  }
  return true;
}

// Return a summary report of the stream settings of status.
std::string USBDevCtl::Report(bool status, bool verbose) const {
  return "";
}

void USBDevCtl::DumpCtlTransfer(struct libusb_transfer *xfr) const {
  const void *buf = reinterpret_cast<void *>(xfr->buffer);
  std::cout << "Buffer " << buf << " length " << xfr->length
            << " => actual length " << xfr->actual_length << std::endl;
  buffer_dump(stdout, xfr->buffer, xfr->actual_length);
}

// Retrieving of IN traffic from device.
bool USBDevCtl::ServiceIN() {
  // Do we still have IN traffic to retrieve
  if (bytes_recvd_ < transfer_bytes_) {
    uint32_t to_fetch = transfer_bytes_ - bytes_recvd_;

    // TODO: It would be good to exercise arbitrary-length data transfers in time!
    if (to_fetch > maxPacketSize_) {
      to_fetch = maxPacketSize_;
    }

    // Ensure that we have enough space available for a full packet; the device
    // software decides upon the length of each packet.
    uint8_t *space;
    bool ok = ProvisionSpace(&space, kSetupSize + maxPacketSize_, true);
    if (ok) {
      // Ensure that we have a transfer descriptor available for use.
      if (!xfr_) {
        xfr_ = dev_->AllocTransfer(0U);
        if (!xfr_) {
          return false;
        }
      }

      // TODO: for now we'll just go via a local buffer as a starting step...
      uint8_t *space = buffer_;

      // SETUP fields
      uint8_t bmRequestType = 0xC2U;
      uint8_t bRequest = USBDevice::kVendorGetData;
      // Note: for now we have no need for the wValue field
      uint16_t wValue = 0U;
      uint16_t wIndex = epIn_ & 0xfU;  // Note: lose the IN direction bit
      uint16_t wLength = to_fetch;

      libusb_fill_control_setup(space, bmRequestType, bRequest, wValue, wIndex, wLength);

      // Note: we must direct the SETUP packet at the OUT endpoint
      assert(!((uintptr_t)buffer_ & 1U));
      dev_->FillControlTransfer(xfr_, epOut_, space, CbStubIN, this, kCtlTimeout);

      int rc = dev_->SubmitTransfer(xfr_);
      if (rc < 0) {
        return dev_->ErrorUSB("ERROR: Submitting IN transfer", rc);
      }
      activeState_ = CtlInActive;
    } else {
      activeState_ = CtlIdle;
    }
  } else {
    activeState_ = CtlIdle;
  }
//std::cout << PrefixID() << " servin " << activeState_ << " rx " << bytes_recvd_ << " trans " << transfer_bytes_ << std::endl;

  return true;
}

// Sending of OUT traffic to device.
bool USBDevCtl::ServiceOUT() {
  // Do we have any data ready to send?
  uint8_t *data;
  uint32_t num_bytes = DataAvailable(&data, true);
//printf("wr %u rd %u end %u\n", buf_.wr_idx, buf_.rd_idx, buf_.end_idx);
  if (num_bytes > 0U) {
    // Supply details of the single OUT packet
    if (!xfr_) {
      xfr_ = dev_->AllocTransfer(0U);
      if (!xfr_) {
        // Stream is not operational.
        return false;
      }
    }

    // TODO: obviously we would like to exercise longer data transfers
    uint32_t to_send = num_bytes;
    if (to_send >= USBDevice::kDevDataMaxPacketSize) {
      to_send = USBDevice::kDevDataMaxPacketSize;
    }

    // SETUP fields
    uint8_t bmRequestType = 0x42U;
    uint8_t bRequest = USBDevice::kVendorPutData;
    uint16_t wValue = 0U;
    uint16_t wIndex = epOut_ & 0xfU;
    uint16_t wLength = to_send;

    libusb_fill_control_setup(buffer_, bmRequestType, bRequest, wValue, wIndex, wLength);

    // The OUT DATA must immediately follow the SETUP DATA
    memcpy(buffer_ + kSetupSize, data, to_send);

    assert(!((uintptr_t)buffer_ & 1U));
    dev_->FillControlTransfer(xfr_, epOut_, buffer_, CbStubOUT, this, kCtlTimeout);

    int rc = dev_->SubmitTransfer(xfr_);
    if (rc < 0) {
      return dev_->ErrorUSB("ERROR: Submitting OUT transfer", rc);
    }
    activeState_ = CtlOutActive;
  } else {
    // Nothing to propagate at this time
    activeState_ = CtlIdle;
  }
//std::cout << PrefixID() << " servout " << activeState_ << " num " << num_bytes << std::endl;
  // Stream remains operational, even if it presently has no work on the OUT
  // side.
  return true;
}

bool USBDevCtl::Service() {
//printf("Failed_%u active %u xfr %p\n", failed_, activeState_, xfr_);
//printf(" buffer rd %u wr %u end %u\n", buf_.rd_idx, buf_.wr_idx, buf_.end_idx);
  if (failed_) {
    return false;
  }
  // Propagate any OUT traffic first, to release space in the circular buffer.
  if (activeState_ == CtlIdle && !ServiceOUT()) {
    return false;
  }
  // (Re)start Isochronous IN traffic if not already in progress.
  if (activeState_ == CtlIdle && !ServiceIN()) {
    return false;
  }
  return true;
}

// Callback function supplied to libusb for IN transfers.
void USBDevCtl::CallbackIN(struct libusb_transfer *xfr) {
  switch (xfr->status) {
    case LIBUSB_TRANSFER_ERROR:
      if (CanSchedule()) {
        // Note: It seems that we can have our transfers rejected at any point
        // because our device is unable to accept them when the device is busy
        // with other streams, and the host controller gives up after just 30us
        // having struck out 3 times!
        int rc = dev_->SubmitTransfer(xfr_);
        if (rc < 0) {
          std::cerr << PrefixID() << "ERROR: Re-submitting IN transfer" << rc << std::endl;
          failed_ = true;
        }
      } else {
        activeState_ = CtlIdle;
      }
      return;

    case LIBUSB_TRANSFER_COMPLETED:
      break;

    default:
      std::cerr << PrefixID() << " Invalid/unexpected IN transfer status "
                << xfr->status << std::endl;
      std::cerr << "length " << xfr->length << " actual " << xfr->actual_length
                << std::endl;
      failed_ = true;
      return;
  }

  if (verbose_) {
    std::cout << PrefixID() << "CallbackIN xfr " << xfr << std::endl;
    DumpCtlTransfer(xfr);
  }

  // Skip past the SETUP DATA packet at the start of the buffer
  uint8_t *sp = &xfr->buffer[kSetupSize];
  // The `actual_length` field excludes the size of the SETUP packet
  int nrecvd = xfr->actual_length;

  assert(nrecvd <= (int)USBDevice::kDevDataMaxPacketSize);

  // Note: We have already ensured that enough contiguous space is available
  uint8_t *dp;
  uint32_t space_bytes = SpaceAvailable(&dp, true);
  assert((ssize_t)space_bytes >= nrecvd);

  // Update the circular buffer with the amount of data that we've received
  AddData(sp, (uint32_t)nrecvd);

  if (!SigReceived()) {
    if (nrecvd > 0 && !SigReceived()) {
      uint32_t dropped = SigDetect(&sig_, dp, (uint32_t)nrecvd);

      // Consume stream signature, rather than propagating it to the output
      // side.
      if (SigReceived()) {
        SigProcess(sig_);
        dropped += sizeof(usbdev_stream_sig_t);
      }

      // Skip past any dropped bytes, including the signature, so that if there
      // are additional bytes we may process them.
      nrecvd = ((uint32_t)nrecvd > dropped) ? ((uint32_t)nrecvd - dropped) : 0;
      dp += dropped;

      if (dropped) {
        ConsumeData(dropped);
      }

      // TODO: bodge!
//      printf("Adjusting byte counts from %u %u\n", bytes_recvd_, bytes_sent_);
      bytes_sent_ = 0U;
      bytes_recvd_ = 0U;
    }
  }

  bool ok = true;
  if (nrecvd > 0) {
    // Check the received LFSR-generated byte(s) and combine them with the
    // output of our host-side LFSR
    ok = ProcessData(dp, nrecvd);
  }

  if (ok) {
    if (CanSchedule()) {
      // Attempt to set up another IN transfer
// TODO: we perhaps want to be setting up an OUT transfer here rather than another IN?
      failed_ = !ServiceIN();
    } else {
      activeState_ = CtlIdle;
    }
  } else {
    failed_ = true;
  }
}

// Callback function supplied to libusb for OUT transfers.
void USBDevCtl::CallbackOUT(struct libusb_transfer *xfr) {
  switch (xfr->status) {
    case LIBUSB_TRANSFER_ERROR:
      if (CanSchedule()) {
        // Note: It seems that we can have our transfers rejected at any point
        // because our device is unable to accept them when the device is busy
        // with other streams, and the host controller gives up after just 30us
        // having struck out 3 times!
        int rc = dev_->SubmitTransfer(xfr_);
        if (rc < 0) {
          std::cerr << PrefixID() << "ERROR: Re-submitting OUT transfer" << rc << std::endl;
          failed_ = true;
        }
      } else {
        activeState_ = CtlIdle;
      }
      return;

    case LIBUSB_TRANSFER_COMPLETED:
      break;

    default:
      std::cerr << PrefixID() << " Invalid/unexpected OUT transfer status "
                << xfr->status << std::endl;
      std::cerr << "length " << xfr->length << " actual " << xfr->actual_length
                << std::endl;
      failed_ = true;
      return;
  }

  if (verbose_) {
    std::cout << PrefixID() << "CallbackOUT xfr " << xfr << std::endl;
    DumpCtlTransfer(xfr);
  }

  // Note: we're not expecting any truncation on OUT transfers but only
  // `length` includes the SETUP bytes
  assert(xfr->actual_length + kSetupSize == (uint32_t)xfr->length);
  ConsumeData(xfr->actual_length);

  if (CanSchedule()) {
    // Attempt to set up another OUT transfer
    failed_ = !ServiceOUT();
  } else {
    activeState_ = CtlIdle;
  }
}
