// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Implementation of the following two HCI-specified tables:
//
// - Device Address Table.
// - Device Characteristics Table.
//
// These two tables need to be accessed by both software and hardware, with the hardware
// having higher priority and stalling the software access slightly when necessary.
// No requirement for sub-word support.
//
// Each table entry spans multiple bus words but there is no requirement for sub-word accesses
// to be supported.
//
// TODO: At the very least the above comments need updating; this is a sketch with a
// still-unresolved issue about the number of write strobes and the sw-side data width.

module i3c_dxt
  import prim_ram_1p_pkg::*;
#(
  // Width of each entry, in bits.
  parameter int unsigned EntryWidth = 72,
  parameter int unsigned NumEntries = 32,
  // Additional `Out Of Band` bits for port B, since it has a variable response time.
  parameter int unsigned OOBWidth   = 1,
  // Derived parameters.
  localparam int unsigned IdxW = $clog2(NumEntries)
) (
  input                   clk_i,
  input                   rst_ni,

  // Port A (highest priority).
  // - hardware interface; reads/writes full entries.
  input                   a_re_i,
  input                   a_we_i,
  input        [IdxW-1:0] a_idx_i,
  input  [EntryWidth-1:0] a_wdata_i,
  output [EntryWidth-1:0] a_rdata_o,

  // Port B (lowest priority).
  // - software interface has bus width read/write access, which translate to memory sub-words
  //   since the memory entries are wider.
  input                   b_req_i,
  output                  b_gnt_o,
  input                   b_we_i,
  input        [IdxW-1:0] b_idx_i,
  input  [EntryWidth-1:0] b_wmask_i,
  input  [EntryWidth-1:0] b_wdata_i,
  input    [OOBWidth-1:0] b_oob_i,
  output                  b_rvalid_o,
  output [EntryWidth-1:0] b_rdata_o,
  output   [OOBWidth-1:0] b_oob_o,

  input  ram_1p_cfg_t     cfg_i,
  output ram_1p_cfg_rsp_t cfg_rsp_o
);

  logic [EntryWidth-1:0] rdata_full;
  logic a_req;

  assign a_req = a_re_i | a_we_i;
  assign b_gnt_o = ~a_req;

  prim_ram_1p #(
    .Width            (EntryWidth),
    .Depth            (NumEntries),
    .DataBitsPerMask  (1)
  ) u_ram (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    .req_i      (a_req | b_req_i),
    .write_i    (a_req ? a_we_i    : b_we_i),
    .addr_i     (a_req ? a_idx_i   : b_idx_i),
    .wdata_i    (a_req ? a_wdata_i : b_wdata_i),
    .wmask_i    (a_req ? '1        : b_wmask_i),
    .rdata_o    (rdata_full),

    .cfg_i      (cfg_i),
    .cfg_rsp_o  (cfg_rsp_o)
  );

  logic [OOBWidth-1:0] b_oob;
  logic b_rvalid;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      b_rvalid  <= 1'b0;
      b_oob     <= '0;
    end else begin
      // Read data is returned a cycle after the read request.
      b_rvalid  <= &{b_req_i, ~b_we_i, ~a_req};
      // Ensure that the OOB data is propagated too.
      if (b_req_i & ~a_req) begin
        b_oob   <= b_oob_i;
      end
    end
  end
  assign b_rvalid_o = b_rvalid;
  assign b_oob_o    = b_oob;
  assign {a_rdata_o, b_rdata_o} = {2{rdata_full}};

endmodule
