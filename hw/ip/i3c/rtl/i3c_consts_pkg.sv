// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

package i3c_consts_pkg;

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

endpackage
