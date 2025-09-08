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
  parameter int unsigned NumSDALanes = 1,
  parameter int unsigned NumDATEntries = i3c_pkg::NumDATEntries,
  parameter int unsigned NumDCTEntries = i3c_pkg::NumDCTEntries,

  // Derived parameters.
  localparam int unsigned DATAddrW = $clog2(NumDATEntries),
  localparam int unsigned DCTAddrW = $clog2(NumDCTEntries)
) (
  // Clock and reset for system interface.
  input                     clk_i,
  input                     rst_ni,

  // Clock and reset for `always on` bus monitoring.
  input                     clk_aon_i,
  input                     rst_aon_ni,

  // Configuration.
  input  i3c_reg2hw_t       reg2hw,
  output i3c_hw2reg_t       hw2reg,

  // HCI XFER_DATA_PORT interface to buffer.
  input                     xfer_req_i,
  output                    xfer_gnt_o,
  input                     xfer_we_i,
  input   [DataWidth/8-1:0] xfer_wmask_i,
  input     [DataWidth-1:0] xfer_wdata_i,
  output                    xfer_rvalid_o,
  output    [DataWidth-1:0] xfer_rdata_o,
  output              [1:0] xfer_rerror_o,

  // HCI Device Address Table interface.
  input                     sw_dat_req_i,
  output                    sw_dat_gnt_o,
  input                     sw_dat_we_i,
  input        [DATAddrW:0] sw_dat_addr_i,
  input     [DataWidth-1:0] sw_dat_wdata_i,
  output                    sw_dat_rvalid_o,
  output    [DataWidth-1:0] sw_dat_rdata_o,

  // HCI Device Characteristics Table interface
  input                     sw_dct_req_i,
  output                    sw_dct_gnt_o,
  input                     sw_dct_we_i,
  input      [DCTAddrW+1:0] sw_dct_addr_i,
  input     [DataWidth-1:0] sw_dct_wdata_i,
  output                    sw_dct_rvalid_o,
  output    [DataWidth-1:0] sw_dct_rdata_o,

  // Direct software interface to buffer.
  // TODO: INTRODUCE _ ?
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
  output                    intr_hci_o,

  // DFT-related controls.
  input  ram_1p_cfg_t       ram_cfg_i,
  output ram_1p_cfg_rsp_t   ram_cfg_rsp_o,
  input  ram_1p_cfg_t       dat_cfg_i,
  output ram_1p_cfg_rsp_t   dat_cfg_rsp_o,
  input  ram_1p_cfg_t       dct_cfg_i,
  output ram_1p_cfg_rsp_t   dct_cfg_rsp_o,
  input                     mbist_en_i,
  input                     scan_clk_i,
  input                     scan_rst_ni,
  input mubi4_t             scanmode_i
);

  // DFT-related signals.
  // TODO: sync?
  wire scanmode = prim_mubi_pkg::mubi4_test_true_strict(scanmode_i);

  // Interrupt signals.
  i3c_stby_cr_intr_t stby_cr_interrupts;
  i3c_pio_intr_t pio_interrupts;
  i3c_hc_intr_t hc_interrupts;

  logic activate_patt_det;
  logic patt_det_active;
  logic hdr_mode;

  // TODO: We shall probably need to delay activation in response to the target enable being
  // asserted until such a time that we can be sure that the bus is idle.

  // TODO: Just for testing HDR Exit signalling, but still in SDR.
  assign hdr_mode = 1'b1;
  assign activate_patt_det = 1'b1;
  // assign hdr_mode = reg2hw.ctrl.hdr_ddr_en;

  logic ctrl_tx_enable;
  logic ctrl_rx_enable;
  logic targ_tx_enable;
  logic targ_rx_enable;
  logic ctrl_sw_reset;
  logic targ_sw_reset;
  logic ddr_enable;

  // Software resets for Controller and Target logic.
  assign ctrl_sw_reset = reg2hw.ctrl.ctrl_reset.qe & reg2hw.ctrl.ctrl_reset.q;
  assign targ_sw_reset = reg2hw.ctrl.targ_reset.qe & reg2hw.ctrl.targ_reset.q;

  // TODO: Temporary, obviously
  assign ctrl_tx_enable = reg2hw.ctrl.ctrl_tx_en;
  assign ctrl_rx_enable = reg2hw.ctrl.ctrl_rx_en;
  assign targ_tx_enable = reg2hw.ctrl.targ_tx_en;
  assign targ_rx_enable = reg2hw.ctrl.targ_rx_en;
  assign ddr_enable = reg2hw.ctrl.hdr_ddr_en;

  // Control signals from Controller logic.
  logic gen_hdr_exit;
  logic gen_hdr_sr;
  assign gen_hdr_exit = 1'b0;
  assign gen_hdr_sr = 1'b0;

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

  // Controller state signals.
  logic ac_current_own;

  // Controller logic.
  i3c_controller #(
    .DataWidth  (DataWidth)
  ) u_controller (
    // Clock and reset for system interface.
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),

    // Clock and reset for `always on` bus monitoring.
    .clk_aon_i        (clk_aon_i),
    .rst_aon_ni       (rst_aon_ni),

    // Control inputs.
    .enable_i         (ctrl_tx_enable | ctrl_rx_enable),
    // TODO: Probably replace this with the HCI RESET_CONTROL signals.
    .sw_reset_i       (ctrl_sw_reset),

    // Configuration settings.
    .reg2hw_i         (reg2hw),

    // State information, presented via HCI.
    .ac_current_own_o (ac_current_own),

    // Command Descriptor queue.
    .cmd_desc_full_o  (),
    .cmd_desc_write_i (reg2hw.command_queue_port.qe),
    .cmd_desc_wdata_i (reg2hw.command_queue_port.q),

    // Response Descriptor queue.
    .rsp_desc_avail_o (),
    .rsp_desc_read_i  (reg2hw.response_queue_port.re),
    .rsp_desc_rdata_o (hw2reg.response_queue_port.d),

    // IBI Status/Data queue.
    .ibi_stat_avail_o (),
    .ibi_stat_read_i  (reg2hw.ibi_port.re),
    .ibi_stat_rdata_o (hw2reg.ibi_port.d),

    // HCI Device Address Table interface.
    .sw_dat_req_i     (sw_dat_req_i),
    .sw_dat_gnt_o     (sw_dat_gnt_o),
    .sw_dat_we_i      (sw_dat_we_i),
    .sw_dat_addr_i    (sw_dat_addr_i),
    .sw_dat_wdata_i   (sw_dat_wdata_i),
    .sw_dat_rvalid_o  (sw_dat_rvalid_o),
    .sw_dat_rdata_o   (sw_dat_rdata_o),

    // HCI Device Characteristics Table interface.
    .sw_dct_req_i     (sw_dct_req_i),
    .sw_dct_gnt_o     (sw_dct_gnt_o),
    .sw_dct_we_i      (sw_dct_we_i),
    .sw_dct_addr_i    (sw_dct_addr_i),
    .sw_dct_wdata_i   (sw_dct_wdata_i),
    .sw_dct_rvalid_o  (sw_dct_rvalid_o),
    .sw_dct_rdata_o   (sw_dct_rdata_o),

    // Interrupt signals.
    .intr_hc_o        (hc_interrupts),
    .intr_pio_o       (pio_interrupts),
    .intr_stby_cr_o   (stby_cr_interrupts),

    // DFT-related signals.
    .dat_cfg_i        (dat_cfg_i),
    .dat_cfg_rsp_o    (dat_cfg_rsp_o),
    .dct_cfg_i        (dct_cfg_i),
    .dct_cfg_rsp_o    (dct_cfg_rsp_o)
  );

  // Target logic.
  i3c_target u_target (
    // Clock and reset for system interface.
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    // Clock and reset for `always on` bus monitoring.
    .clk_aon_i    (clk_aon_i),
    .rst_aon_ni   (rst_aon_ni),

    // Control inputs.
    .enable_i     (targ_tx_enable | targ_rx_enable),
    .sw_reset_i   (targ_sw_reset),

    // Configuration inputs.
    .reg2hw_i     (reg2hw)
  );

  // Buffer configuration.
  i3c_pkg::bufcfg_t ctrl_txbuf_cfg, targ_txbuf_cfg;  // Transmission.
  i3c_pkg::bufcfg_t ctrl_rxbuf_cfg, targ_rxbuf_cfg;  // Reception.
  i3c_pkg::bufcfg_t xfer_txbuf_cfg, xfer_rxbuf_cfg;  // HCI XFER_DATA_PORT.
  i3c_pkg::bufcfg_t swbuf_cfg;                       // Direct software access.

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

  // XFER_DATA_PORT access.
  assign xfer_txbuf_cfg.min   = BufAddrW'(reg2hw.ctrl_txbuf_config.min_addr.q);
  assign xfer_txbuf_cfg.max   = BufAddrW'(reg2hw.ctrl_txbuf_config.max_addr.q);
  assign xfer_txbuf_cfg.curr  = BufAddrW'(reg2hw.ctrl_txbuf_wptr.q);
  assign xfer_txbuf_cfg.limit = BufAddrW'(reg2hw.ctrl_txbuf_rptr.q);

  assign xfer_rxbuf_cfg.min   = BufAddrW'(reg2hw.ctrl_rxbuf_config.min_addr.q);
  assign xfer_rxbuf_cfg.max   = BufAddrW'(reg2hw.ctrl_rxbuf_config.max_addr.q);
  assign xfer_rxbuf_cfg.curr  = BufAddrW'(reg2hw.ctrl_rxbuf_rptr.q);
  assign xfer_rxbuf_cfg.limit = BufAddrW'(reg2hw.ctrl_rxbuf_wptr.q);

  // Construct an artificial configuration for the direct software buffer access.
  assign swbuf_cfg.min   = '0;
  assign swbuf_cfg.max   = '1;
  assign swbuf_cfg.curr  = BufAddrW'(swbuf_addr_i);
  assign swbuf_cfg.limit = '1;

  // Update buffer status.
  // - pointer writes from the hardware.
  // - software reset mechanism rewinds both `rptr` and `wptr` to their associated `min` value.
  wire buf_sw_clear = reg2hw.buffer_ctrl.qe & reg2hw.buffer_ctrl.q;
  i3c_pkg::bufupd_t ctrl_txbuf_upd, targ_txbuf_upd;
  i3c_pkg::bufupd_t ctrl_rxbuf_upd, targ_rxbuf_upd;
  i3c_pkg::bufupd_t xfer_txbuf_upd, xfer_rxbuf_upd;

  assign hw2reg.ctrl_txbuf_rptr.de = ctrl_txbuf_upd.valid | buf_sw_clear;
  assign hw2reg.ctrl_txbuf_rptr.d  = buf_sw_clear ? 16'(ctrl_txbuf_cfg.min)
                                                  : 16'(ctrl_txbuf_upd.next);
  assign hw2reg.ctrl_rxbuf_wptr.de = ctrl_rxbuf_upd.valid | buf_sw_clear;
  assign hw2reg.ctrl_rxbuf_wptr.d  = buf_sw_clear ? 16'(ctrl_rxbuf_cfg.min)
                                                  : 16'(ctrl_rxbuf_upd.next);
  assign hw2reg.targ_txbuf_rptr.de = targ_txbuf_upd.valid | buf_sw_clear;
  assign hw2reg.targ_txbuf_rptr.d  = buf_sw_clear ? 16'(targ_txbuf_cfg.min)
                                                  : 16'(targ_txbuf_upd.next);
  assign hw2reg.targ_rxbuf_wptr.de = targ_rxbuf_upd.valid | buf_sw_clear;
  assign hw2reg.targ_rxbuf_wptr.d  = buf_sw_clear ? 16'(targ_rxbuf_cfg.min)
                                                  : 16'(targ_rxbuf_upd.next);

  assign hw2reg.ctrl_txbuf_wptr.de = xfer_txbuf_upd.valid | buf_sw_clear;
  assign hw2reg.ctrl_txbuf_wptr.d  = buf_sw_clear ? 16'(ctrl_txbuf_cfg.min)
                                                  : 16'(xfer_txbuf_upd.next);
  assign hw2reg.ctrl_rxbuf_rptr.de = xfer_rxbuf_upd.valid | buf_sw_clear;
  assign hw2reg.ctrl_rxbuf_rptr.d  = buf_sw_clear ? 16'(ctrl_rxbuf_cfg.min)
                                                  : 16'(xfer_rxbuf_upd.next);

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
    .xfer_txbuf_cfg_i   (xfer_txbuf_cfg),
    .xfer_rxbuf_cfg_i   (xfer_rxbuf_cfg),
    .swbuf_cfg_i        (swbuf_cfg),
    // Update buffer status.
    .ctrl_txbuf_upd_o   (ctrl_txbuf_upd),
    .targ_txbuf_upd_o   (targ_txbuf_upd),
    .ctrl_rxbuf_upd_o   (ctrl_rxbuf_upd),
    .xfer_txbuf_upd_o   (xfer_txbuf_upd),
    .xfer_rxbuf_upd_o   (xfer_rxbuf_upd),
    .targ_rxbuf_upd_o   (targ_rxbuf_upd),

    // HCI XFER_DATA_PORT interface to buffer.
    .xfer_req_i         (xfer_req_i),
    .xfer_gnt_o         (xfer_gnt_o),
    .xfer_we_i          (xfer_we_i),
    .xfer_wmask_i       (xfer_wmask_i),
    .xfer_wdata_i       (xfer_wdata_i),
    .xfer_rvalid_o      (xfer_rvalid_o),
    .xfer_rdata_o       (xfer_rdata_o),
    .xfer_rerror_o      (xfer_rerror_o),

    // Direct software interface to buffer.
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

  // TODO:
  i3c_dtype_e ctrl_txbuf_rtype;
//  assign ctrl_txbuf_rtype = I3CType_CommandWord;
// TODO: Development.
 assign ctrl_txbuf_rtype = I3CType_DWORD;
//
// assign ctrl_txbuf_rtype = I3CType_Address;

  // Controller Transceiver.
  i3c_controller_trx #(
    .NumSDALanes (NumSDALanes)
  ) u_ctrl_trx (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),

    .sw_reset_i     (ctrl_sw_reset),

    .tx_enable_i    (ctrl_tx_enable),
    .rx_enable_i    (ctrl_rx_enable),
    .ddr_enable_i   (ddr_enable),

    // Timing parameters.
    // TODO: These all need sizing and ratifying of course.
    .clkdiv_i       (reg2hw.ctrl_clk_config.clkdiv.q),
    .tcas_i         ({8'b0, reg2hw.ctrl_clk_config.tcas.q}),
    .tcbp_i         ({8'b0, reg2hw.ctrl_clk_config.tcbp.q}),

    // Control inputs.
    .gen_hdr_exit_i (gen_hdr_exit),
    .gen_hdr_sr_i   (gen_hdr_sr),

    // Buffer reading.
    .buf_rd_o       (ctrl_txbuf_rd),
    .buf_rtype_i    (ctrl_txbuf_rtype),
    .buf_rdata_i    (ctrl_txbuf_rdata),
    .buf_empty_i    (ctrl_txbuf_empty),

    // Buffer writing.
    .buf_wr_o       (ctrl_rxbuf_wr),
    .buf_wmask_o    (ctrl_rxbuf_wmask),
    .buf_wdata_o    (ctrl_rxbuf_wdata),

    // I3C I/O signaling.
    .scl_o          (ctrl_scl_o),
    .scl_pp_en_o    (ctrl_scl_pp_en_o),
    .scl_od_en_o    (ctrl_scl_od_en_o),
    .sda_i          (ctrl_sda_i),
    .sda_o          (ctrl_sda_o),
    .sda_pp_en_o    (ctrl_sda_pp_en_o),
    .sda_od_en_o    (ctrl_sda_od_en_o),

    // Pullup enables.
    .scl_pu_en_o    (ctrl_scl_pu_en_o),
    .sda_pu_en_o    (ctrl_sda_pu_en_o)
  );

  // Target Transceiver.
  i3c_target_trx #(
    .NumSDALanes  (NumSDALanes),
    .DataWidth    (DataWidth)
  ) u_targ_trx (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),

    .sw_reset_i         (targ_sw_reset),

    .tx_enable_i        (targ_tx_enable),
    .rx_enable_i        (targ_rx_enable),
    .ddr_enable_i       (ddr_enable),

    // Secondary Controller address and validity indicator.
    .ctrl_addr_i        (reg2hw.controller_device_addr.dynamic_addr),
    .ctrl_addr_valid_i  (reg2hw.controller_device_addr.dynamic_addr_valid),
    // Target addresses and masks.
    .targ_addr0_mask_i  (reg2hw.targ_addr_mask.mask0),
    .targ_addr0_match_i (reg2hw.targ_addr_match.addr0),
    .targ_addr1_mask_i  (reg2hw.targ_addr_mask.mask1),
    .targ_addr1_match_i (reg2hw.targ_addr_match.addr1),

    // HDR pattern detection.
    .hdr_exit_det_i     (hdr_exit_det),
    .hdr_restart_det_i  (hdr_restart_det),

    // Buffer reading.
    .buf_rd_o           (targ_txbuf_rd),
    .buf_rdata_i        (targ_txbuf_rdata),
    .buf_empty_i        (targ_txbuf_empty),

    // Buffer writing.
    .buf_wr_o           (targ_rxbuf_wr),
    .buf_wmask_o        (targ_rxbuf_wmask),
    .buf_wdata_o        (targ_rxbuf_wdata),

    // I3C I/O signaling.
    .scl_i              (targ_scl_i),
    .sda_i              (targ_sda_i),
    .sda_o              (targ_sda_o),
    .sda_pp_en_o        (targ_sda_pp_en_o),
    .sda_od_en_o        (targ_sda_od_en_o),

    // DFT-related controls.
    .scanmode_i         (scanmode)
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

    // Control inputs.
    .enable_i           (targ_rx_enable),
    // Configuration.
    // TODO: This rst_action is a quasi-static input to be used in SCL-driven logic.
    .rst_action_i       (reg2hw.stby_cr_ccc_config_rstact_params.rst_action),

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
    .scanmode_i         (scanmode)
  );

  // TODO: Dummy status information.
  assign hw2reg.status.d = '0;

  // Master HCI interrupt; asserted when any HCI interrupt is asserted.
  logic intr_hc, intr_pio, int_stby_cr;
  prim_intr_hw #(.IntrT("Status")) u_intr_hci (
    .clk_i                  (clk_i),
    .rst_ni                 (rst_ni),

    .event_intr_i           (intr_hc | intr_pio | intr_stby_cr),

    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.d),

    .intr_o                 (intr_hci_o)
  );

  // Hardware only sets interrupt status bits, never clears.
  assign {hw2reg.intr_status.sched_cmd_missed_tick_stat.d,
          hw2reg.intr_status.hc_err_cmd_seq_timeout_stat.d,
          hw2reg.intr_status.hc_warn_cmd_seq_stall_stat.d,
          hw2reg.intr_status.hc_seq_cancel_stat.d,
          hw2reg.intr_status.hc_internal_err_stat.d} = '1;
  assign {hw2reg.pio_intr_status.transfer_err_stat.d,
          hw2reg.pio_intr_status.transfer_abort_stat.d,
          hw2reg.pio_intr_status.resp_ready_stat.d,
          hw2reg.pio_intr_status.cmd_queue_ready_stat.d,
          hw2reg.pio_intr_status.ibi_status_thld_stat.d,
          hw2reg.pio_intr_status.rx_thld_stat.d,
          hw2reg.pio_intr_status.tx_thld_stat.d} = '1;
  assign {hw2reg.stby_cr_intr_status.ccc_fatal_rstdaa_err_stat.d,
          hw2reg.stby_cr_intr_status.ccc_unhandled_nack_stat.d,
          hw2reg.stby_cr_intr_status.ccc_param_modified_stat.d,
          hw2reg.stby_cr_intr_status.stby_cr_op_rstact_stat.d,
          hw2reg.stby_cr_intr_status.stby_cr_accept_err_stat.d,
          hw2reg.stby_cr_intr_status.stby_cr_accept_ok_stat.d,
          hw2reg.stby_cr_intr_status.stby_cr_accept_nacked_stat.d,
          hw2reg.stby_cr_intr_status.stby_cr_dyn_addr_stat.d,
          hw2reg.stby_cr_intr_status.crr_response_stat.d,
          hw2reg.stby_cr_intr_status.acr_handoff_err_m3_stat.d,
          hw2reg.stby_cr_intr_status.acr_handoff_err_fail_stat.d,
          hw2reg.stby_cr_intr_status.acr_handoff_ok_primed_stat.d,
          hw2reg.stby_cr_intr_status.acr_handoff_ok_remain_stat.d} = '1;

  // General Host Controller interrupts.
  i3c_intr #(.Width(5)) u_hc_intr (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .event_i      (hc_interrupts),
 
    .status_en_i  ({reg2hw.intr_status_enable.sched_cmd_missed_tick_stat_en.q,
                    reg2hw.intr_status_enable.hc_err_cmd_seq_timeout_stat_en.q,
                    reg2hw.intr_status_enable.hc_warn_cmd_seq_stall_stat_en.q,
                    reg2hw.intr_status_enable.hc_seq_cancel_stat_en.q,
                    reg2hw.intr_status_enable.hc_internal_err_stat_en.q}),
    .set_status_o ({hw2reg.intr_status.sched_cmd_missed_tick_stat.de,
                    hw2reg.intr_status.hc_err_cmd_seq_timeout_stat.de,
                    hw2reg.intr_status.hc_warn_cmd_seq_stall_stat.de,
                    hw2reg.intr_status.hc_seq_cancel_stat.de,
                    hw2reg.intr_status.hc_internal_err_stat.de}),
    // TODO: Need to ascertain whether HCI force signals are level or pulse.
    .force_i      ({reg2hw.intr_force.sched_cmd_missed_tick_force.qe,
                    reg2hw.intr_force.hc_err_cmd_seq_timeout_force.qe,
                    reg2hw.intr_force.hc_warn_cmd_seq_stall_force.qe,
                    reg2hw.intr_force.hc_seq_cancel_force.qe,
                    reg2hw.intr_force.hc_internal_err_force.qe}),
    .status_i     ({reg2hw.intr_status.sched_cmd_missed_tick_stat.q,
                    reg2hw.intr_status.hc_err_cmd_seq_timeout_stat.q,
                    reg2hw.intr_status.hc_warn_cmd_seq_stall_stat.q,
                    reg2hw.intr_status.hc_seq_cancel_stat.q,
                    reg2hw.intr_status.hc_internal_err_stat.q}),
    .signal_en_i  ({reg2hw.intr_signal_enable.sched_cmd_missed_tick_signal_en.q,
                    reg2hw.intr_signal_enable.hc_err_cmd_seq_timeout_signal_en.q,
                    reg2hw.intr_signal_enable.hc_warn_cmd_seq_stall_signal_en.q,
                    reg2hw.intr_signal_enable.hc_seq_cancel_signal_en.q,
                    reg2hw.intr_signal_enable.hc_internal_err_signal_en.q}),

    .intr_o       (intr_hc)    
  );

  // PIO interrupts.
  i3c_intr #(.Width(7)) u_pio_intr (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .event_i      (pio_interrupts),
 
    .status_en_i  ({reg2hw.pio_intr_status_enable.transfer_err_stat_en.q,
                    reg2hw.pio_intr_status_enable.transfer_abort_stat_en.q,
                    reg2hw.pio_intr_status_enable.resp_ready_stat_en.q,
                    reg2hw.pio_intr_status_enable.cmd_queue_ready_stat_en.q,
                    reg2hw.pio_intr_status_enable.ibi_status_thld_stat_en.q,
                    reg2hw.pio_intr_status_enable.rx_thld_stat_en.q,
                    reg2hw.pio_intr_status_enable.tx_thld_stat_en.q}),
    .set_status_o ({hw2reg.pio_intr_status.transfer_err_stat.de,
                    hw2reg.pio_intr_status.transfer_abort_stat.de,
                    hw2reg.pio_intr_status.resp_ready_stat.de,
                    hw2reg.pio_intr_status.cmd_queue_ready_stat.de,
                    hw2reg.pio_intr_status.ibi_status_thld_stat.de,
                    hw2reg.pio_intr_status.rx_thld_stat.de,
                    hw2reg.pio_intr_status.tx_thld_stat.de}),
    .force_i      ({reg2hw.pio_intr_force.transfer_err_force.q,
                    reg2hw.pio_intr_force.transfer_abort_force.q,
                    reg2hw.pio_intr_force.resp_ready_force.q,
                    reg2hw.pio_intr_force.cmd_queue_ready_force.q,
                    reg2hw.pio_intr_force.ibi_thld_force.q,
                    reg2hw.pio_intr_force.rx_thld_force.q,
                    reg2hw.pio_intr_force.tx_thld_force.q}),
    .status_i     ({reg2hw.pio_intr_status.transfer_err_stat.q,
                    reg2hw.pio_intr_status.transfer_abort_stat.q,
                    reg2hw.pio_intr_status.resp_ready_stat.q,
                    reg2hw.pio_intr_status.cmd_queue_ready_stat.q,
                    reg2hw.pio_intr_status.ibi_status_thld_stat.q,
                    reg2hw.pio_intr_status.rx_thld_stat.q,
                    reg2hw.pio_intr_status.tx_thld_stat.q}),
    .signal_en_i  ({reg2hw.pio_intr_signal_enable.transfer_err_signal_en.q,
                    reg2hw.pio_intr_signal_enable.transfer_abort_signal_en.q,
                    reg2hw.pio_intr_signal_enable.resp_ready_signal_en.q,
                    reg2hw.pio_intr_signal_enable.cmd_queue_ready_signal_en.q,
                    reg2hw.pio_intr_signal_enable.ibi_status_thld_signal_en.q,
                    reg2hw.pio_intr_signal_enable.rx_thld_signal_en.q,
                    reg2hw.pio_intr_signal_enable.tx_thld_signal_en.q}),

    .intr_o       (intr_pio)
  );

  // Secondary Controller interrupts.
  i3c_intr #(.Width(13)) u_stby_cr_intr (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    .event_i      (stby_cr_interrupts),
 
    .status_en_i  ('1),  // No INTR_STATUS_ENABLE for these interrupts.
    .set_status_o ({hw2reg.stby_cr_intr_status.ccc_fatal_rstdaa_err_stat.de,
                    hw2reg.stby_cr_intr_status.ccc_unhandled_nack_stat.de,
                    hw2reg.stby_cr_intr_status.ccc_param_modified_stat.de,
                    hw2reg.stby_cr_intr_status.stby_cr_op_rstact_stat.de,
                    hw2reg.stby_cr_intr_status.stby_cr_accept_err_stat.de,
                    hw2reg.stby_cr_intr_status.stby_cr_accept_ok_stat.de,
                    hw2reg.stby_cr_intr_status.stby_cr_accept_nacked_stat.de,
                    hw2reg.stby_cr_intr_status.stby_cr_dyn_addr_stat.de,
                    hw2reg.stby_cr_intr_status.crr_response_stat.de,
                    hw2reg.stby_cr_intr_status.acr_handoff_err_m3_stat.de,
                    hw2reg.stby_cr_intr_status.acr_handoff_err_fail_stat.de,
                    hw2reg.stby_cr_intr_status.acr_handoff_ok_primed_stat.de,
                    hw2reg.stby_cr_intr_status.acr_handoff_ok_remain_stat.de}),
    .force_i      ({reg2hw.stby_cr_intr_force.ccc_fatal_rstdaa_err_force.q,
                    reg2hw.stby_cr_intr_force.ccc_unhandled_nack_force.q,
                    reg2hw.stby_cr_intr_force.ccc_param_modified_force.q,
                    reg2hw.stby_cr_intr_force.stby_cr_op_rstact_force.q,
                    reg2hw.stby_cr_intr_force.stby_cr_accept_err_force.q,
                    reg2hw.stby_cr_intr_force.stby_cr_accept_ok_force.q,
                    reg2hw.stby_cr_intr_force.stby_cr_accept_nacked_force.q,
                    reg2hw.stby_cr_intr_force.stby_cr_dyn_addr_force.q,
                    reg2hw.stby_cr_intr_force.crr_response_force.q,
                    4'h0}),  // acr_handoff interrupts do not have force controls.
    .status_i     ({reg2hw.stby_cr_intr_status.ccc_fatal_rstdaa_err_stat.q,
                    reg2hw.stby_cr_intr_status.ccc_unhandled_nack_stat.q,
                    reg2hw.stby_cr_intr_status.ccc_param_modified_stat.q,
                    reg2hw.stby_cr_intr_status.stby_cr_op_rstact_stat.q,
                    reg2hw.stby_cr_intr_status.stby_cr_accept_err_stat.q,
                    reg2hw.stby_cr_intr_status.stby_cr_accept_ok_stat.q,
                    reg2hw.stby_cr_intr_status.stby_cr_accept_nacked_stat.q,
                    reg2hw.stby_cr_intr_status.stby_cr_dyn_addr_stat.q,
                    reg2hw.stby_cr_intr_status.crr_response_stat.q,
                    reg2hw.stby_cr_intr_status.acr_handoff_err_m3_stat.q,
                    reg2hw.stby_cr_intr_status.acr_handoff_err_fail_stat.q,
                    reg2hw.stby_cr_intr_status.acr_handoff_ok_primed_stat.q,
                    reg2hw.stby_cr_intr_status.acr_handoff_ok_remain_stat.q}),
    .signal_en_i  ({reg2hw.stby_cr_intr_signal_enable.ccc_fatal_rstdaa_err_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.ccc_unhandled_nack_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.ccc_param_modified_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.stby_cr_op_rstact_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.stby_cr_accept_err_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.stby_cr_accept_ok_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.stby_cr_accept_nacked_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.stby_cr_dyn_addr_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.crr_response_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.acr_handoff_err_m3_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.acr_handoff_err_fail_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.acr_handoff_ok_primed_signal_en.q,
                    reg2hw.stby_cr_intr_signal_enable.acr_handoff_ok_remain_signal_en.q}),

    .intr_o       (intr_stby_cr)
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

  // HCI information fields.
  // - `BASE` is at offset 'h100 within the I3C block register space.
  // - offsets are specified relative to that base.
  localparam bit [11:0] BASE_OFFSET = 12'h100;
  assign hw2reg.hci_version.d = 'h120;
  assign hw2reg.hc_control.mode_selector.de            = 1'b0;  // Use the reset value.
  assign hw2reg.hc_control.mode_selector.d             = 1'b1;
  assign hw2reg.hc_control.data_byte_order_mode.de     = 1'b0;  // Use the reset value.
  assign hw2reg.hc_control.data_byte_order_mode.d      = 1'b0;
  // Host Controller Capabilities.
  assign hw2reg.hc_capabilities.sg_capability_dc_en.d  = 1'b0;
  assign hw2reg.hc_capabilities.sg_capability_ibi_en.d = 1'b0;
  assign hw2reg.hc_capabilities.sg_capability_cr_en.d  = 1'b0;
  assign hw2reg.hc_capabilities.cmd_size.d             = 2'b00;
  assign hw2reg.hc_capabilities.schedule_commands_en.d = 1'b1;
  assign hw2reg.hc_capabilities.ibi_credit_count_en.d  = 1'b1;
  assign hw2reg.hc_capabilities.ibi_data_abort_en.d    = 1'b1;
  assign hw2reg.hc_capabilities.cmd_ccc_defbyte.d      = 1'b1;
  assign hw2reg.hc_capabilities.hdr_ts_en.d            = 1'b1;
  assign hw2reg.hc_capabilities.hdr_ddr_en.d           = 1'b1;
  assign hw2reg.hc_capabilities.standby_cr_cap.d       = 1'b1;
  assign hw2reg.hc_capabilities.auto_command.d         = 1'b1;
  assign hw2reg.hc_capabilities.combo_command.d        = 1'b1;
  // Active Controller indication.
  assign hw2reg.present_state.d = ac_current_own;
  // Addressing offsets.
  assign hw2reg.pio_section_offset.d              = 16'(I3C_COMMAND_QUEUE_PORT_OFFSET) -
                                                    16'(BASE_OFFSET);
  assign hw2reg.dat_section_offset.entry_size.d   = 'h0;
  assign hw2reg.dat_section_offset.table_size.d   = 7'(NumDATEntries);
  assign hw2reg.dat_section_offset.table_offset.d = I3C_DAT_OFFSET - BASE_OFFSET;
  assign hw2reg.dct_section_offset.entry_size.d   = 'h0;
  assign hw2reg.dct_section_offset.table_size.d   = 7'(NumDCTEntries);
  assign hw2reg.dct_section_offset.table_offset.d = I3C_DCT_OFFSET - BASE_OFFSET;
  assign hw2reg.ring_headers_section_offset.d     = '0;
  assign hw2reg.ext_caps_section_offset.d         = '0;
  // Internal Control Command Subtype Support.
  assign hw2reg.int_ctrl_cmds_en.icc_support.d         = 1'b0;
  assign hw2reg.int_ctrl_cmds_en.mipi_cmds_supported.d = 'h18;
  // Device Context Base Address (applicable only to DMA).
  assign hw2reg.dev_ctx_base_lo.d      = '0;
  assign hw2reg.dev_ctx_base_hi.d      = '0;
  assign hw2reg.dev_ctx_sg.blp.d       = 1'b0;
  assign hw2reg.dev_ctx_sg.list_size.d = '0;
  // Report the dimensions of queues and data buffers.
  assign hw2reg.queue_size.tx_data_buffer_size.d     = 8'(reg2hw.ctrl_txbuf_config.size_val.q);
  assign hw2reg.queue_size.rx_data_buffer_size.d     = 8'(reg2hw.ctrl_rxbuf_config.size_val.q);
  assign hw2reg.queue_size.ibi_status_size.d         = 8'(IBIQueueDWORDs);
  assign hw2reg.queue_size.cr_queue_size.d           = 8'(CmdQueueEntries);
  assign hw2reg.alt_queue_size.ext_ibi_queue_en.d    = 1'b0;
  assign hw2reg.alt_queue_size.alt_resp_queue_en.d   = 1'b1;
  assign hw2reg.alt_queue_size.alt_resp_queue_size.d = 8'(RspQueueEntries);

endmodule
