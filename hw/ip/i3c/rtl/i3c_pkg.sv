// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

package i3c_pkg;

  localparam int unsigned BufAddrW = 9;

  typedef struct packed {
    logic [BufAddrW-1:0] min;
    logic [BufAddrW-1:0] max;
    logic [BufAddrW-1:0] curr;
    logic [BufAddrW-1:0] limit;
  } bufcfg_t;

  typedef struct packed {
    logic                valid;
    logic [BufAddrW-1:0] next;
  } bufupd_t;

endpackage
