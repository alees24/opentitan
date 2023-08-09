// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
#ifndef OPENTITAN_SW_HOST_TESTS_USBDEV_USBDEV_STREAM_USBDEV_CTL_H_
#define OPENTITAN_SW_HOST_TESTS_USBDEV_USBDEV_STREAM_USBDEV_CTL_H_
#include <queue>

#include "usb_device.h"
#include "usbdev_stream.h"

class USBDevCtl : public USBDevStream {
 public:
  USBDevCtl(USBDevice *dev, unsigned id, uint32_t transfer_bytes,
            bool retrieve, bool check, bool send, bool verbose)
      : USBDevStream(id, transfer_bytes, retrieve, check, send, verbose),
        dev_(dev),
        failed_(false),
        activeState_(CtlIdle),
        xfr_(nullptr) {}
  /**
   * Open an Interrupt connection to specified device interface.
   *
   * @param  interface  Interface number.
   * @return The success of the operation.
   */
  bool Open(unsigned interface);
  /**
   * Close this Interrupt stream.
   */
  virtual void Close(bool wait = false);

  virtual bool Resume();
  /**
   * Return a summary report of the stream settings or status.
   *
   * @param  status    Indicates whether settings or status requested.
   * @param  verbose   true iff a more verbose report is required.
   * @return Status report
   */
  virtual std::string Report(bool status = false, bool verbose = false) const;
  /**
   * Service this Interrupt stream.
   *
   * @return true iff the stream is still operational.
   */
  virtual bool Service();

 private:
  /**
   * Diagnostic utility function to display the content of libusb Control Transfer.
   *
   * @param  xfr     The Interrupt transfer to be displayed.
   */
  void DumpCtlTransfer(struct libusb_transfer *xfr) const;
  /**
   * Retrieving of IN traffic from device.
   *
   * @return true iff the stream is still operational.
   */
  bool ServiceIN();
  /**
   * Sending of OUT traffic to device.
   *
   * @return true iff the stream is still operational.
   */
  bool ServiceOUT();
  /**
   * Callback function supplied to libusb for IN transfers; transfer has
   * completed and requires attention.
   *
   * @param  xfr     The transfer that has completed.
   */
  void CallbackIN(struct libusb_transfer *xfr);
  /**
   * Callback function supplied to libusb for OUT transfers; transfer has
   * completed and requires attention.
   *
   * @param  xfr     The transfer that has completed.
   */
  void CallbackOUT(struct libusb_transfer *xfr);
  /**
   * Stub callback function supplied to libusb for IN transfers.
   *
   * @param  xfr     The transfer that has completed.
   */
  static void LIBUSB_CALL CbStubIN(struct libusb_transfer *xfr);
  /**
   * Stub callback function supplied to libusb for OUT transfers.
   *
   * @param  xfr     The transfer that has completed.
   */
  static void LIBUSB_CALL CbStubOUT(struct libusb_transfer *xfr);

  // USB device.
  USBDevice *dev_;

  // The number of the interface being used by this stream.
  unsigned interface_;

  // Has this stream experienced a failure?
  bool failed_;

  // Activity state
  //
  // Note: for Control streams since there can only be a single Control Transfer
  // in progress at a time, we require only a single transfer descriptor and
  // the INput and OUTput cannot be active simultaneously
  enum {
    CtlIdle = 0,
    CtlInActive,
    CtlOutActive
  } activeState_;

  // Current Control Transfer
  struct libusb_transfer *xfr_;

  // Maximum packet size for this stream.
  uint8_t maxPacketSize_;

  // Endpoint numbers used by this stream.
  uint8_t epIn_;
  uint8_t epOut_;

  // Stream signature.
  // Note: this is constructed piecemeal as the bytes are received from the
  // device.
  usbdev_stream_sig_t sig_;

  // Packet buffer; this is required for OUT transfers because the SETUP packet
  // and the OUT data field must be contiguous for libusb
  alignas(uint16_t) uint8_t buffer_[8U + USBDevice::kDevDataMaxPacketSize];

  // Size of SETUP DATA packet in bytes (specification requirement)
  static constexpr unsigned kSetupSize = 8U;

  // TODO: timeout parameters need consideration.
  //  static constexpr unsigned kIsoTimeout = 0U;
  static constexpr unsigned kCtlTimeout = 0U;
};

#endif  // OPENTITAN_SW_HOST_TESTS_USBDEV_USBDEV_STREAM_USBDEV_CTL_H_
