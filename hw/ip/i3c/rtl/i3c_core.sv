// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i3c_core
  import i3c_pkg::*;
  import i3c_reg_pkg::*;
  import prim_mubi_pkg::*;
  import prim_ram_1p_pkg::*;
#(
  parameter int unsigned BufAddrW    = i3c_pkg::BufAddrW,
  parameter int unsigned DataWidth   = 32,
  parameter int unsigned NumSDALanes = 1
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
  input      [BufAddrW-1:0] swbuf_addr_i,
  input   [DataWidth/8-1:0] swbuf_wmask_i,
  input     [DataWidth-1:0] swbuf_wdata_i,
  output                    swbuf_rvalid_o,
  output              [1:0] swbuf_rerror_o,
  output    [DataWidth-1:0] swbuf_rdata_o,

  // I3C Controller I/O signaling.
  input                     ctrl_scl_i,
  output                    ctrl_scl_o,
  output                    ctrl_scl_pp_en_o,
  output                    ctrl_scl_od_en_o,
  input   [NumSDALanes-1:0] ctrl_sda_i,
  output  [NumSDALanes-1:0] ctrl_sda_o,
  output                    ctrl_sda_pp_en_o,
  output                    ctrl_sda_od_en_o,

  // Pullup enables.
  output                    ctrl_scl_pu_en_o,
  output                    ctrl_sda_pu_en_o,

  // High-keeper enables.
  output                    scl_hk_en_o,
  output                    sda_hk_en_o,

  // I3C Target I/O signaling.
  input                     targ_scl_i,
  input   [NumSDALanes-1:0] targ_sda_i,
  output  [NumSDALanes-1:0] targ_sda_o,
  output                    targ_sda_pp_en_o,
  output                    targ_sda_od_en_o,

  // Chip reset request.
  output                    chip_rst_req_o,

  // Interrupts.
  output                    intr_done_o,
  output                    intr_error_o,

  // DFT-related controls.
  input                     mbist_en_i,
  input                     scan_clk_i,
  input                     scan_rst_ni,
  input mubi4_t             scanmode_i
);

  // DFT-related signals.
  // TODO: sync?
  wire scanmode = prim_mubi_pkg::mubi4_test_true_strict(scanmode_i);

  logic activate_patt_det;
  logic patt_det_active;
  logic hdr_mode;

  assign activate_patt_det = 1'b0;
  assign hdr_mode = 1'b0;

  logic ctrl_tx_enable;
  logic ctrl_rx_enable;
  logic targ_tx_enable;
  logic targ_rx_enable;
  logic ddr_enable;

  // TODO: Temporary, obviously
  assign ctrl_tx_enable = reg2hw.ctrl.ctrl_tx_en;
  assign ctrl_rx_enable = reg2hw.ctrl.ctrl_rx_en;
  assign targ_tx_enable = reg2hw.ctrl.targ_tx_en;
  assign targ_rx_enable = reg2hw.ctrl.targ_rx_en;
  assign ddr_enable = reg2hw.ctrl.hdr_ddr_en;

  // Controller TX read access to the buffer.
  logic                   ctrl_txbuf_rd;
  logic   [DataWidth-1:0] ctrl_txbuf_rdata;
  logic                   ctrl_txbuf_empty;

  // Controller RX write access to the buffer.
  logic                   ctrl_rxbuf_wr;
  logic [DataWidth/8-1:0] ctrl_rxbuf_wmask;
  logic   [DataWidth-1:0] ctrl_rxbuf_wdata;

  // Target TX read access to the buffer.
  logic                   targ_txbuf_rd;
  logic   [DataWidth-1:0] targ_txbuf_rdata;
  logic                   targ_txbuf_empty;

  // Target RX write access to the buffer.
  logic                   targ_rxbuf_wr;
  logic [DataWidth/8-1:0] targ_rxbuf_wmask;
  logic   [DataWidth-1:0] targ_rxbuf_wdata;

  // Signals from pattern detector, synchronized to main clock.
  logic hdr_restart_det;
  logic hdr_exit_det;
  logic target_reset;

  // High-keeper enables.
  assign scl_hk_en_o = reg2hw.phy_config.scl_hk_en.q;
  assign sda_hk_en_o = reg2hw.phy_config.sda_hk_en.q;

  // Controller logic.
  i3c_controller u_controller (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .clk_aon_i    (clk_aon_i),
    .rst_aon_ni   (rst_aon_ni),

    .enable_i     (ctrl_tx_enable | ctrl_rx_enable),

    // Pullup enables.
    .scl_pu_en_o  (ctrl_scl_pu_en_o),
    .sda_pu_en_o  (ctrl_sda_pu_en_o)
  );

  // Target logic.
  i3c_target u_target (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .clk_aon_i    (clk_aon_i),
    .rst_aon_ni   (rst_aon_ni),

    .enable_i     (targ_tx_enable | targ_rx_enable)
  );

  // Buffer configuration.
  i3c_pkg::bufcfg_t ctrl_txbuf_cfg, targ_txbuf_cfg;  // Transmission.
  i3c_pkg::bufcfg_t ctrl_rxbuf_cfg, targ_rxbuf_cfg;  // Reception.
  i3c_pkg::bufcfg_t swbuf_cfg;

  assign ctrl_txbuf_cfg.min   = BufAddrW'(reg2hw.ctrl_txbuf_config.min_addr.q);
  assign ctrl_txbuf_cfg.max   = BufAddrW'(reg2hw.ctrl_txbuf_config.max_addr.q);
  assign ctrl_txbuf_cfg.curr  = BufAddrW'(reg2hw.ctrl_txbuf_rptr.q);
  assign ctrl_txbuf_cfg.limit = BufAddrW'(reg2hw.ctrl_txbuf_wptr.q);

  assign ctrl_rxbuf_cfg.min   = BufAddrW'(reg2hw.ctrl_rxbuf_config.min_addr.q);
  assign ctrl_rxbuf_cfg.max   = BufAddrW'(reg2hw.ctrl_rxbuf_config.max_addr.q);
  assign ctrl_rxbuf_cfg.curr  = BufAddrW'(reg2hw.ctrl_rxbuf_wptr.q);
  assign ctrl_rxbuf_cfg.limit = BufAddrW'(reg2hw.ctrl_rxbuf_rptr.q);

  assign targ_txbuf_cfg.min   = BufAddrW'(reg2hw.targ_txbuf_config.min_addr.q);
  assign targ_txbuf_cfg.max   = BufAddrW'(reg2hw.targ_txbuf_config.max_addr.q);
  assign targ_txbuf_cfg.curr  = BufAddrW'(reg2hw.targ_txbuf_rptr.q);
  assign targ_txbuf_cfg.limit = BufAddrW'(reg2hw.targ_txbuf_wptr.q);

  assign targ_rxbuf_cfg.min   = BufAddrW'(reg2hw.targ_rxbuf_config.min_addr.q);
  assign targ_rxbuf_cfg.max   = BufAddrW'(reg2hw.targ_rxbuf_config.max_addr.q);
  assign targ_rxbuf_cfg.curr  = BufAddrW'(reg2hw.targ_rxbuf_wptr.q);
  assign targ_rxbuf_cfg.limit = BufAddrW'(reg2hw.targ_rxbuf_rptr.q);

  // Construct an artificial configuration for the software buffer access.
  assign swbuf_cfg.min   = '0;
  assign swbuf_cfg.max   = '1;
  assign swbuf_cfg.curr  = BufAddrW'(swbuf_addr_i);
  assign swbuf_cfg.limit = '1;

  // Update buffer status.
  i3c_pkg::bufupd_t ctrl_txbuf_upd, targ_txbuf_upd;
  i3c_pkg::bufupd_t ctrl_rxbuf_upd, targ_rxbuf_upd;

  assign hw2reg.ctrl_txbuf_rptr.de = ctrl_txbuf_upd.valid;
  assign hw2reg.ctrl_txbuf_rptr.d  = 16'(ctrl_txbuf_upd.next);
  assign hw2reg.ctrl_rxbuf_wptr.de = ctrl_rxbuf_upd.valid;
  assign hw2reg.ctrl_rxbuf_wptr.d  = 16'(ctrl_rxbuf_upd.next);

  assign hw2reg.targ_txbuf_rptr.de = targ_txbuf_upd.valid;
  assign hw2reg.targ_txbuf_rptr.d  = 16'(targ_txbuf_upd.next);
  assign hw2reg.targ_rxbuf_wptr.de = targ_rxbuf_upd.valid;
  assign hw2reg.targ_rxbuf_wptr.d  = 16'(targ_rxbuf_upd.next);

  // TODO: Placeholder status information.
  assign hw2reg.status.rx_idle.d = '0;
  assign hw2reg.status.tx_idle.d = '0;
  assign hw2reg.buffer_status.rxlvl.d = '0;
  assign hw2reg.buffer_status.txlvl.d = '0;

  // Data buffer for transmission and reception.
  i3c_buffer #(
    .BufAddrW  (BufAddrW),
    .DataWidth (DataWidth)
  ) u_buf (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),

    .ram_cfg_i          (ram_cfg_i),
    .ram_cfg_rsp_o      (ram_cfg_rsp_o),

    // Buffer configuration.
    .ctrl_txbuf_cfg_i   (ctrl_txbuf_cfg),
    .targ_txbuf_cfg_i   (targ_txbuf_cfg),
    .ctrl_rxbuf_cfg_i   (ctrl_rxbuf_cfg),
    .targ_rxbuf_cfg_i   (targ_rxbuf_cfg),
    .swbuf_cfg_i        (swbuf_cfg),
    // Update buffer status.
    .ctrl_txbuf_upd_o   (ctrl_txbuf_upd),
    .targ_txbuf_upd_o   (targ_txbuf_upd),
    .ctrl_rxbuf_upd_o   (ctrl_rxbuf_upd),
    .targ_rxbuf_upd_o   (targ_rxbuf_upd),

    // Software interface to buffer.
    .swbuf_req_i        (swbuf_req_i),
    .swbuf_gnt_o        (swbuf_gnt_o),
    .swbuf_we_i         (swbuf_we_i),
    .swbuf_addr_i       (swbuf_addr_i),
    .swbuf_wmask_i      (swbuf_wmask_i),
    .swbuf_wdata_i      (swbuf_wdata_i),
    .swbuf_rvalid_o     (swbuf_rvalid_o),
    .swbuf_rerror_o     (swbuf_rerror_o),
    .swbuf_rdata_o      (swbuf_rdata_o),

    // Controller TX interface to buffer.
    .ctrl_txbuf_rd_i    (ctrl_txbuf_rd),
    .ctrl_txbuf_rdata_o (ctrl_txbuf_rdata),
    .ctrl_txbuf_empty_o (ctrl_txbuf_empty),

    // Controller RX interface to buffer.
    .ctrl_rxbuf_wr_i    (ctrl_rxbuf_wr),
    .ctrl_rxbuf_wmask_i (ctrl_rxbuf_wmask),
    .ctrl_rxbuf_wdata_i (ctrl_rxbuf_wdata),

    // Target TX interface to buffer.
    .targ_txbuf_rd_i    (targ_txbuf_rd),
    .targ_txbuf_rdata_o (targ_txbuf_rdata),
    .targ_txbuf_empty_o (targ_txbuf_empty),

    // Target RX interface to buffer.
    .targ_rxbuf_wr_i    (targ_rxbuf_wr),
    .targ_rxbuf_wmask_i (targ_rxbuf_wmask),
    .targ_rxbuf_wdata_i (targ_rxbuf_wdata)
  );

  // Controller Transceiver.
  i3c_tx #(
    .NumSDALanes (NumSDALanes)
  ) u_tx (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .tx_enable_i  (ctrl_tx_enable),
    .rx_enable_i  (ctrl_rx_enable),
    .ddr_enable_i (ddr_enable),

    // Timing parameters.
    .clkdiv_i     ('0),
    .tcas_i       (16'h0002),
    .tcbp_i       (16'h0000),

    // Buffer reading.
    .buf_rd_o     (ctrl_txbuf_rd),
    .buf_rdata_i  (ctrl_txbuf_rdata),
    .buf_empty_i  (ctrl_txbuf_empty),

    // Buffer writing.
    .buf_wr_o     (ctrl_rxbuf_wr),
    .buf_wmask_o  (ctrl_rxbuf_wmask),
    .buf_wdata_o  (ctrl_rxbuf_wdata),

    // I3C I/O signaling.
    .scl_o        (ctrl_scl_o),
    .scl_pp_en_o  (ctrl_scl_pp_en_o),
    .scl_od_en_o  (ctrl_scl_od_en_o),
    .sda_i        (ctrl_sda_i),
    .sda_o        (ctrl_sda_o),
    .sda_pp_en_o  (ctrl_sda_pp_en_o),
    .sda_od_en_o  (ctrl_sda_od_en_o)
  );

  // Target Transceiver.
  i3c_rx #(
    .NumSDALanes  (NumSDALanes),
    .DataWidth    (DataWidth)
  ) u_rx (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .tx_enable_i  (targ_tx_enable),
    .rx_enable_i  (targ_rx_enable),
    .ddr_enable_i (ddr_enable),

    // Buffer reading.
    .buf_rd_o     (targ_txbuf_rd),
    .buf_rdata_i  (targ_txbuf_rdata),
    .buf_empty_i  (targ_txbuf_empty),

    // Buffer writing.
    .buf_wr_o     (targ_rxbuf_wr),
    .buf_wmask_o  (targ_rxbuf_wmask),
    .buf_wdata_o  (targ_rxbuf_wdata),

    // I3C I/O signaling.
    .scl_i        (targ_scl_i),
    .sda_i        (targ_sda_i),
    .sda_o        (targ_sda_o),
    .sda_pp_en_o  (targ_sda_pp_en_o),
    .sda_od_en_o  (targ_sda_od_en_o),

    // DFT-related controls.
    .scanmode_i   (scanmode)
  );

  // Pattern detection signals (AON domain).
  logic hdr_restart_det_aon;
  logic hdr_exit_det_aon;
  logic target_reset_aon;
  logic chip_reset_aon;

  // I3C pattern detector:
  // - HDR Exit, HDR Restart and Target Reset.
  // TODO: This may need to be moved outside of the core IP block.
  i3c_patt_detector u_patt_det (
    .rst_aon_ni         (rst_aon_ni),

    // I3C I/O signaling.
    .scl_i              (targ_scl_i),
    .sda_i              (targ_sda_i),

    // State.
    .activate_i         (activate_patt_det),
    .hdr_mode_i         (hdr_mode),
    .active_o           (patt_det_active),

    /// HDR pattern detection.
    .hdr_exit_det_o     (hdr_exit_det_aon),
    .hdr_restart_det_o  (hdr_restart_det_aon),

    .target_reset_o     (target_reset_aon),
    .chip_reset_o       (chip_reset_aon),

    // DFT-related controls.
    .mbist_en_i         (mbist_en_i),
    .scan_clk_i         (scan_clk_i),
    .scan_rst_ni        (scan_rst_ni),
    .scanmode_i         (scanmode)
  );

  // TODO:
  wire event_done  = 1'b0;
  wire event_error = 1'b0;

  // TODO: Placeholder completion interrupt.
  prim_intr_hw #(.Width(1)) intr_done (
    .clk_i,
    .rst_ni,
    .event_intr_i           (event_done),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.done.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.done.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.done.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.done.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.done.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.done.d),
    .intr_o                 (intr_done_o)
  );

  // TODO: Placeholder error interrupt.
  prim_intr_hw #(.Width(1)) intr_error (
    .clk_i,
    .rst_ni,
    .event_intr_i           (event_error),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.error.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.error.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.error.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.error.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.error.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.error.d),
    .intr_o                 (intr_error_o)
  );

`ifdef PULSE_SYNCS
  prim_pulse_sync u_restart_sync (
    .clk_src_i          (),
    
  );

  prim_pulse_sync u_exit_sync (
  );

  prim_pulse_sync u_targ_rst_sync (
  );

  prim_pulse_sync u_chip_rst_sync (
  );
`else
assign hdr_restart_det = hdr_restart_det_aon;
assign hdr_exit_det = hdr_exit_det_aon;
assign target_reset = target_reset_aon;
assign chip_rst_req_o = chip_reset_aon;
`endif

endmodule
