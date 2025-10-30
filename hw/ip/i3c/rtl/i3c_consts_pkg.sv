// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Constants defined by the I3C Specification rather than this IP block.
package i3c_consts_pkg;

  typedef enum logic [6:0] {
    Addr_Broadcast = 7'h7e  // I3C Broadcast Address
  } i3c_addr_e;

  // Common Command Codes
  typedef enum logic [7:0] {
    // Broadcast
    ENECB     = 8'h00,  // Enable Events Command
    DISECB    = 8'h01,  // Disable Events Command

    RSTDAA    = 8'h06,  // Reset Dynamic Address Assignment
    ENTDAA    = 8'h07,  // Enter Dynamic Address Assignment
    DEFTGTS   = 8'h08,  // Define List of Targets
    SETMWLB   = 8'h09,  // Set Max Write Length
    SETMRLB   = 8'h0a,  // Set Max Read Length
    ENTTM     = 8'h0b,  // Enter Test Mode

    ENTHDR0   = 8'h20,  // Enter HDR Mode 0
    ENTHDR1   = 8'h21,  // Enter HDR Mode 1
    ENTHDR2   = 8'h22,  // Enter HDR Mode 2
    ENTHDR3   = 8'h23,  // Enter HDR Mode 3
    ENTHDR4   = 8'h24,  // Enter HDR Mode 4
    ENTHDR5   = 8'h25,  // Enter HDR Mode 5
    ENTHDR6   = 8'h26,  // Enter HDR Mode 6
    ENTHDR7   = 8'h27,  // Enter HDR Mode 7

    SETAASA   = 8'h29,  // Set All Addresses to Static Addresses
    RSTACTB   = 8'h2a,  // Target Reset Action
    DEFGRPA   = 8'h2b,  // Define List of Group Addresses
    RSTGRPAB  = 8'h2c,  // Reset Group Address

    // Directed
    ENEC      = 8'h80,  // Enable Events Command
    DISEC     = 8'h81,  // Disable Events Command

    SETDASA   = 8'h87,  // Set Dynamic Address from Static Address
    SETNEWDA  = 8'h88,  // Set New Dynamic Address
    SETMWL    = 8'h89,  // Set Max Write Length
    SETMRL    = 8'h8a,  // Set Max Read Length
    GETMWL    = 8'h8b,  // Get Max Write Length
    GETMRL    = 8'h8c,  // Get Max Read Length
    GETPID    = 8'h8d,  // Get Provisioned ID
    GETBCR    = 8'h8e,  // Get Bus Characteristics Register
    GETDCR    = 8'h8f,  // Get Device Characteristics Register
    GETSTATUS = 8'h90,  // Get Device Status
    GETACCCR  = 8'h91,  // Get Accept Controller Role
    ENDXFER   = 8'h92,  // Data Transfer Ending Procedure Control

    GETMXDS   = 8'h94,  // Get Max Data Speed
    GETCAPS   = 8'h95,  // Get Optional Feature Capabilities
    SETROUTE  = 8'h96,  // Set Route

    RSTACT    = 8'h9a,  // Target Reset Action
    SETGRPA   = 8'h9b,  // Set Group Address
    RSTGRPA   = 8'h9c   // Reset Group Address
  } i3c_ccc_e;

  // RSTACT Defining Byte Values (5.1.9.3.26)
  typedef enum logic [7:0] {
    RstAct_NoReset         = 8'h00, // No Reset on Target Reset Pattern.
    RstAct_ResetPeripheral = 8'h01, // Reset the I3C Peripheral Only.
    RstAct_ResetTarget     = 8'h02, // Reset the Whole Target.
    RstAct_DebugNetReset   = 8'h03, // Debug Network Adapter Reset.
    RstAct_VirtualTargDet  = 8'h04, // Virtual Target Detect.
    RstAct_ResetPeriphTime = 8'h81, // Return Time to Reset Peripheral.
    RstAct_ResetTargetTime = 8'h82, // Return Time to Reset Whole Target.
    RstAct_ResetDbgNetTime = 8'h83, // Return Time for Debug Network Adapter Reset.
    RstAct_VirtTargIndican = 8'h84  // Return Virtual Target Indication.
  } i3c_rstact_e;

  // I3C Transfer Mode (TCRI 7.1.1.1)
  typedef enum logic [2:0] {
    XferMode_SDR0       = 3'h0,
    XferMode_SDR1       = 3'h1,
    XferMode_SDR2       = 3'h2,
    XferMode_SDR3       = 3'h3,
    XferMode_SDR4       = 3'h4,
    XferMode_HDRTernary = 3'h5,
    XferMode_HDRDDR     = 3'h6
  } i3c_xfer_mode_e;

  // I2C Transfer Mode (TCRI 7.1.1.1)
  typedef enum logic [2:0] {
    XferMode_I2CFM      = 3'h0,
    XferMode_I2CFMPlus  = 3'h1,
    XferMode_I2CUDR1    = 3'h2,
    XferMode_I2CUDR2    = 3'h3,
    XferMode_I2CUDR3    = 3'h4
  } i2c_xfer_mode_e;

  // CMD_ATTR field of Command Descriptor (TCRI 7.1.2)
  typedef enum logic [2:0] {
    CmdAttr_RegTransfer    = 3'h0,
    CmdAttr_ImmTransfer    = 3'h1,
    CmdAttr_AddrAssignment = 3'h2,
    CmdAttr_ComboTransfer  = 3'h3,
    CmdAttr_InternalCtrl   = 3'h7
  } i3c_cmd_attr_e;

  // Error Status Codes (TCRI 6.4.1)
  typedef enum logic [3:0] {
    ErrStatus_OK = 4'h0,
    ErrStatus_CRC,
    ErrStatus_Parity,
    ErrStatus_Frame,
    ErrStatus_AddrHeader,
    ErrStatus_NACK,
    ErrStatus_OVL,
    ErrStatus_I3CShortReadErr,
    ErrStatus_HCAborted,
    ErrStatus_BusAborted = 4'h9,
    ErrStatus_NotSupported,
    ErrStatus_AbortedWithCRC
  } i3c_err_status_e;

  // This value is overloaded, having different meanings for I2C and I3C transfers.
  localparam i3c_err_status_e ErrStatus_I2CWrDataNack = i3c_err_status_e'(4'h9);

endpackage
