// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i3c_target
  import i3c_pkg::*;
  import i3c_reg_pkg::*;
#(
  // Number of target(s) or target group(s) presented simultaneously on the I3C bus, including the
  // Secondary Controller Role.
  parameter int unsigned NumTargets = 2,
  parameter int unsigned DataWidth = 32
) (
  // Clock and reset for system interface.
  input                     clk_i,
  input                     rst_ni,

  // Clock and reset for `always on` bus monitoring.
  input                     clk_aon_i,
  input                     rst_aon_ni,

  // Control inputs.
  input                     enable_i,
  input                     sw_reset_i,

  // Configuration settings.
  input  i3c_reg2hw_t       reg2hw_i,

  // State information, presented via TTI.

  // Buffer reading.
  output                    buf_rd_o,
  input     [DataWidth-1:0] buf_rdata_i,
  input                     buf_empty_i,

  // Buffer writing.
  // TODO: Do we want byte-level access anyway?!
  output                    buf_wr_o,
  output  [DataWidth/8-1:0] buf_wmask_o,
  output    [DataWidth-1:0] buf_wdata_o,

  // Requests/state information to transceiver.
  // TODO: We are expecting to replicate this per target/command at some point.
  output logic              trx_dvalid_o[NumTargets],
  input  logic              trx_dready_i[NumTargets],
  output i3c_targ_trx_req_t trx_dreq_o[NumTargets],

  // I3C clock signal from the controller.
  input                     scl_ni,

  // Response from transceiver logic.
  input  logic              trx_rtoggle_i,
  input  i3c_targ_trx_rsp_t trx_rsp_i
);

  // Synchronize the response data into the IP clock domain.
  logic trx_valid, trx_ready;
  i3c_targ_trx_rsp_t trx_rsp;

  i3c_sync_data #(.Width($bits(i3c_targ_trx_rsp_t))) u_trx_rsp (
    // Source clock domain.
    .clk_src_i    (1'b0),  // Not used.
    .rst_src_ni   (1'b1),  // Not used.
    .src_toggle_i (trx_rtoggle_i),
    .src_toggle_o (),  // Not used.
    .src_data_i   (trx_rsp_i),
    // Destination clock domain.
    .clk_dst_i    (clk_i),
    .rst_dst_ni   (rst_ni),
    .dst_valid_o  (trx_valid),
    .dst_ready_i  (trx_ready),
    .dst_data_o   (trx_rsp)
  );

  // Accept responses immediately.
  assign trx_ready = trx_valid;

  // Send all data into the message buffer for now.
  logic        wvalid;
  logic  [3:0] wmask;
  logic [31:0] wdata;
  assign buf_wr_o    = wvalid;
  assign buf_wmask_o = wmask;
  assign buf_wdata_o = wdata;

  i3c_dword_buffer u_dword_buf (
    .clk_i      (clk_i),
    .rst_ni   (rst_ni),

    // Input SDR bytes/HDR-DDR Data Words.
    .valid_i  (trx_valid & trx_ready),
    .flush_i  (1'b0),
    .dtype_i  (trx_rsp_i.dtype),
    .data_i   (trx_rsp_i.wdata),

    // Output DWORDs.
    .valid_o  (wvalid),
    .mask_o   (wmask),
    .data_o   (wdata)
  );

// TODO:
i3c_targ_trx_req_t src_data;
assign src_data.mask     = {7{reg2hw_i.stby_cr_device_addr.dynamic_addr_valid}};
assign src_data.addr     =    reg2hw_i.stby_cr_device_addr.dynamic_addr;
assign src_data.rdata_nq = 10'h3ab;
assign src_data.rdata_pq = 10'h123;

  // Present read status information for each supported target.
  for (genvar t = 0; t < NumTargets; t++) begin : rxinfo
    // TODO: temporary code for bring up
    logic src_toggle_out;
    logic src_toggle;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) src_toggle  <= 1'b1;
      else if (src_toggle_out == src_toggle)
        src_toggle <= !src_toggle;
    end

    i3c_sync_data #(
      .Width          ($bits(i3c_targ_trx_req_t)),
      .EnSrcToggleOut (1)
    ) u_rxinfo (
      // IP block domain.
      .clk_src_i    (clk_i),
      .rst_src_ni   (rst_ni),
      .src_toggle_i (src_toggle),
      .src_toggle_o (src_toggle_out),
      .src_data_i   (src_data),
      // SCL-clocked domain.
      .clk_dst_i    (scl_ni),
      .rst_dst_ni   (rst_ni),
      .dst_valid_o  (trx_dvalid_o[t]),
      .dst_ready_i  (trx_dready_i[t]),
      .dst_data_o   (trx_dreq_o[t])
    );
  end

  // TODO: We cannot read yet.
  assign buf_rd_o = 1'b0;

endmodule
