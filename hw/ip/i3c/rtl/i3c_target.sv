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

  // Prefetch requests from transceiver.
  input                     trx_ptoggle_i,
  input i3c_targ_trx_pre_t  trx_pre_i,

  // Requests/state information to transceiver.
  output logic              trx_dvalid_o[NumTargets],
  input                     trx_dready_i[NumTargets],
  output i3c_targ_trx_req_t trx_dreq_o[NumTargets],

  // I3C clock signal from the controller.
  input                     scl_ni,

  // Response from transceiver logic.
  input                     trx_rtoggle_i,
  input  i3c_targ_trx_rsp_t trx_rsp_i
);

  // Synchronize the prefetch request into the IP clock domain.
  logic trx_pvalid, trx_pready;
  i3c_targ_trx_pre_t trx_pre;

  i3c_sync_data #(.Width($bits(i3c_targ_trx_pre_t))) u_trx_pre (
    // Source clock domain.
    .clk_src_i    (1'b0),  // Not used.
    .rst_src_ni   (1'b1),  // Not used.
    .src_toggle_i (),
    .src_toggle_o (),
    .src_data_i   (),
    // Destination clock domain.
    .clk_dst_i    (clk_i),
    .rst_dst_ni   (rst_ni),
    .dst_valid_o  (trx_pvalid),
    .dst_ready_i  (trx_pready),
    .dst_data_o   (trx_pre)
  );

  // Synchronize the response data into the IP clock domain.
  logic trx_rvalid, trx_rready;
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
    .dst_valid_o  (trx_rvalid),
    .dst_ready_i  (trx_rready),
    .dst_data_o   (trx_rsp)
  );

  // Accept responses immediately.
  assign trx_rready = trx_rvalid;

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
    .valid_i  (trx_rvalid & trx_rready),
    .flush_i  (1'b0),
    .dtype_i  (trx_rsp_i.dtype),
    .data_i   (trx_rsp_i.wdata),

    // Output DWORDs.
    .valid_o  (wvalid),
    .mask_o   (wmask),
    .data_o   (wdata)
  );

  logic src_toggle_out[NumTargets];
  logic src_toggle[NumTargets];
  i3c_targ_trx_req_t src_data;

  task collect_cmd();
  endtask

  // TODO: The _trx interface wants only 8- or 16-bit data, but we still want to collect DWORDs
  // from the message buffer, so there will need to be some decoupling.

  // TODO: This is undoubtedly going to be subject to refinement...
  task send_data(bit first, logic [15:0] d);
    // Odd-indexed bits are clocked out on the negative edge of SCL, to be ready for the rising edge
    // at the receiver.
    src_data.rdata_nq[9] <= 1'b1;
    src_data.rdata_pq[9] <= !first;
    for (int unsigned b = 0; b < 8; b++) begin
      src_data.rdata_nq[b+1] <= d[2*b+1];
      src_data.rdata_pq[b+1] <= d[2*b];
    end
    // TODO: I think we don't even need to include these; replaced with parity bits.
    src_data.rdata_nq[0] <= 1'b0;
    src_data.rdata_pq[0] <= 1'b0;
  endtask

  task send_crc();
    // Preamble bits are 2'b01, followed by 4'hC to indicate a CRC Word.
    src_data.rdata_nq[9:7] <= {1'b0, 1'b1, 1'b0};
    src_data.rdata_pq[9:7] <= {1'b1, 1'b1, 1'b0};
    // The ensuing data bits will be replaced by the CRC-5 value.
  endtask

  task nack_cmd();
    src_data.rdata_nq[9] <= 1'b1;
    src_data.rdata_pq[9] <= 1'b1;
  endtask

  // TODO: Work out what we can avoid replicating per target, given that all targets share a message
  // buffer and only one can be processing a command at a time.
  logic [3:0] tx_cnt;
  logic tx_first;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tx_first  <= 1'b1;
      tx_cnt    <= 'h0;
      src_data.targ_id  <= 1'b1;
    end else if (tx_first || src_toggle_out[1] == src_toggle[1]) begin
      case (tx_cnt)
        4'h0: send_data(1'b1, 32'hbaad);
        4'h1: send_data(1'b0, 32'hf00d);
        4'h2: send_data(1'b0, 32'hdead);
        4'h3: send_data(1'b0, 32'hbeef);
        4'h4: send_data(1'b0, 32'h0123);
        4'h5: send_data(1'b0, 32'h4567);
        4'h6: send_data(1'b0, 32'h89ab);
        4'h7: send_data(1'b0, 32'hcdef);
        4'h8: send_crc();
        // Nothing to send.
        default: nack_cmd();
      endcase
      tx_first  <= 1'b0;
      tx_cnt    <= tx_cnt + 'b1;
    end
  end

  // Present read status information for each supported target.
  // TODO: rxinfo here is poorly-name; looks like device info.
  for (genvar t = 0; t < NumTargets; t++) begin : rxinfo
    // TODO: temporary code for bring up
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) src_toggle[t] <= 1'b1;
      else if (src_toggle_out[t] == src_toggle[t])
        src_toggle[t] <= !src_toggle[t];
    end

    i3c_sync_data #(
      .Width          ($bits(i3c_targ_trx_req_t)),
      .EnSrcToggleOut (1)
    ) u_rxinfo (
      // IP block domain.
      .clk_src_i    (clk_i),
      .rst_src_ni   (rst_ni),
      .src_toggle_i (src_toggle[t]),
      .src_toggle_o (src_toggle_out[t]),
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
