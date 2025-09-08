// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i3c_core
  import i3c_reg_pkg::*;
  import prim_mubi_pkg::*;
  import prim_ram_1p_pkg::*;
#(
  parameter int unsigned BufAddrWidth = 9,
  parameter int unsigned DataWidth = 32
) (
  // Clock and reset for system interface.
  input                     clk_i,
  input                     rst_ni,

  // Clock and reset for `always on` bus monitoring.
  input                     clk_aon_i,
  input                     rst_aon_ni,

  input  ram_1p_cfg_t       ram_cfg_i,
  output ram_1p_cfg_rsp_t   ram_cfg_rsp_o,

  // Configuration.
  input  i3c_reg2hw_t       reg2hw,
  output i3c_hw2reg_t       hw2reg,

  // Software interface to buffer.
  input                     swbuf_req_i,
  output                    swbuf_gnt_o,
  input                     swbuf_we_i,
  input  [BufAddrWidth-1:0] swbuf_addr_i,
  input   [DataWidth/8-1:0] swbuf_wmask_i,
  input     [DataWidth-1:0] swbuf_wdata_i,
  output                    swbuf_rvalid_o,
  output              [1:0] swbuf_rerror_o,
  output    [DataWidth-1:0] swbuf_rdata_o,

  // I3C I/O signaling.
  input                     scl_i,
  output                    scl_o,
  output                    scl_en_o,
  input                     sda_i,
  output                    sda_o,
  output                    sda_en_o,
  output                    sda_od_en_o,

  // DFT-related controls.
  input                     mbist_en_i,
  input                     scan_clk_i,
  input                     scan_rst_ni,
  input mubi4_t             scanmode_i
);

  logic activate_patt_det;
  logic patt_det_active;
  logic hdr_mode;

  assign activate_patt_det = 1'b0;
  assign hdr_mode = 1'b0;

  logic tx_enable;
  logic rx_enable;
  logic ddr_enable;

  assign tx_enable = 1'b1;
  assign rx_enable = 1'b1;
  assign ddr_enable = 1'b1;

  // Transmitter read access to the buffer.
  logic                   txbuf_rd;
  logic   [DataWidth-1:0] txbuf_rdata;
  logic                   txbuf_empty;

  // Receiver write access to the buffer.
  logic                   rxbuf_wr;
  logic [DataWidth/8-1:0] rxbuf_wmask;
  logic   [DataWidth-1:0] rxbuf_wdata;

  // Controller logic.
  i3c_controller u_controller (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .clk_aon_i    (clk_aon_i),
    .rst_aon_ni   (rst_aon_ni),

    .enable_i     (reg2hw.ctrl.ctrl_en)
  );

  // Target logic.
  i3c_target u_target (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .clk_aon_i    (clk_aon_i),
    .rst_aon_ni   (rst_aon_ni),

    .enable_i     (reg2hw.ctrl.targ_en)
  );

  // Data buffer for transmission and reception.
  i3c_buffer #(
    .BufAddrWidth (BufAddrWidth),
    .DataWidth    (DataWidth)
  ) u_buf (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),

    .ram_cfg_i      (ram_cfg_i),
    .ram_cfg_rsp_o  (ram_cfg_rsp_o),

    // Software interface to buffer.
    .swbuf_req_i    (swbuf_req_i),
    .swbuf_gnt_o    (swbuf_gnt_o),
    .swbuf_we_i     (swbuf_we_i),
    .swbuf_addr_i   (swbuf_addr_i),
    .swbuf_wmask_i  (swbuf_wmask_i),
    .swbuf_wdata_i  (swbuf_wdata_i),
    .swbuf_rvalid_o (swbuf_rvalid_o),
    .swbuf_rerror_o (swbuf_rerror_o),
    .swbuf_rdata_o  (swbuf_rdata_o),

    // Transmitter interface to buffer.
    .txbuf_rd_i     (txbuf_rd),
    .txbuf_rdata_o  (txbuf_rdata),
    .txbuf_empty_o  (txbuf_empty),

    // Receiver interface to buffer.
    .rxbuf_wr_i     (rxbuf_wr),
    .rxbuf_wmask_i  (rxbuf_wmask),
    .rxbuf_wdata_i  (rxbuf_wdata)
  );

  // Transmitter (used by both Controller and Target).
  i3c_tx u_tx (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .tx_enable_i  (tx_enable),
    .ddr_enable_i (ddr_enable),

    // Timing parameters.
    .clkdiv_i     ('0),
    .tcas_i       (16'h0001),
    .tcbp_i       (16'h0000),

    // Buffer reading.
    .buf_rd_o     (txbuf_rd),
    .buf_rdata_i  (txbuf_rdata),
    .buf_empty_i  (txbuf_empty),

    // I3C I/O signaling.
    .scl_o        (scl_o),
    .scl_en_o     (scl_en_o),
    .sda_o        (sda_o),
    .sda_en_o     (sda_en_o),
    .sda_od_en_o  (sda_od_en_o)
  );

  // Receiver (used by both Controller and Target).
  i3c_rx u_rx (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .clk_aon_i    (clk_aon_i),
    .rst_aon_ni   (rst_aon_ni),

    .rx_enable_i  (rx_enable),
    .ddr_enable_i (ddr_enable),

    // Buffer writing.
    .buf_wr_o     (rxbuf_wr),
    .buf_wmask_o  (rxbuf_wmask),
    .buf_wdata_o  (rxbuf_wdata),

    // I3C I/O signaling.
    .scl_i        (scl_i),
    .sda_i        (sda_i)
  );

  // I3C pattern detector:
  // - HDR Exit, HDR Restart and Target Reset.
  // TODO: This may need to be moved outside of the core IP block.
  i3c_patt_detector u_patt_det (
    .rst_ni             (rst_ni),

    // I3C I/O signaling.
    .scl_i              (scl_i),
    .sda_i              (sda_i),

    // State.
    .activate_i         (activate_patt_det),
    .hdr_mode_i         (hdr_mode),
    .active_o           (patt_det_active),

    /// HDR pattern detection.
    .hdr_exit_det_o     (),
    .hdr_restart_det_o  (),

    .target_reset_o     (),
    .chip_reset_o       (),

    // DFT-related controls.
    .mbist_en_i         (mbist_en_i),
    .scan_clk_i         (scan_clk_i),
    .scan_rst_ni        (scan_rst_ni),
    .scanmode_i         (scanmode_i)
  );

endmodule
