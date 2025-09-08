// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C top level

`include "prim_assert.sv"

module i3c
  import i3c_reg_pkg::*;
#(
  parameter logic [NumAlerts-1:0]           AlertAsyncOn              = '1,
  // Number of cycles of differential skew tolerated on the alert signal.
  parameter int unsigned                    AlertSkewCycles           = 1,
  // Number of cycles of external delay on the received input signals (SCL/SDA).
  parameter int unsigned                    InputDelayCycles          = 0,
  parameter int unsigned                    NumSDALanes               = 1,
  parameter bit                             EnableRacl                = 1'b0,
  parameter bit                             RaclErrorRsp              = EnableRacl,
  parameter top_racl_pkg::racl_policy_sel_t RaclPolicySelVec[NumRegs] = '{NumRegs{0}}
) (
  input                                     clk_i,
  input                                     rst_ni,

  input                                     clk_aon_i,
  input                                     rst_aon_ni,

  input  prim_ram_1p_pkg::ram_1p_cfg_t      ram_cfg_i,
  output prim_ram_1p_pkg::ram_1p_cfg_rsp_t  ram_cfg_rsp_o,

  // Register interface
  input  tlul_pkg::tl_h2d_t                 tl_i,
  output tlul_pkg::tl_d2h_t                 tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // RACL interface
  input  top_racl_pkg::racl_policy_vec_t    racl_policies_i,
  output top_racl_pkg::racl_error_log_t     racl_error_o,

  // I3C Controller I/O signaling.
  input                                     cio_ctrl_scl_i,
  output logic                              cio_ctrl_scl_o,
  output logic                              cio_ctrl_scl_pp_en_o,
  output logic                              cio_ctrl_scl_od_en_o,

  input                   [NumSDALanes-1:0] cio_ctrl_sda_i,
  output logic            [NumSDALanes-1:0] cio_ctrl_sda_o,
  output logic                              cio_ctrl_sda_pp_en_o,
  output logic                              cio_ctrl_sda_od_en_o,

  // Pullup enables for open drain intervals.
  output logic                              cio_ctrl_scl_pu_en_o,
  output logic                              cio_ctrl_sda_pu_en_o,

  // High-keeper enables.
  output logic                              cio_scl_hk_en_o,
  output logic                              cio_sda_hk_en_o,

  // I3C Target I/O signaling.
  input                                     cio_targ_scl_i,
  input                   [NumSDALanes-1:0] cio_targ_sda_i,
  output logic            [NumSDALanes-1:0] cio_targ_sda_o,
  output logic                              cio_targ_sda_pp_en_o,
  output logic                              cio_targ_sda_od_en_o,

  // Chip reset request.
  output logic                              cio_chip_rst_req_o,

  // Interrupt-signaling to hardware.
  output logic                              lsio_trigger_o,

  // Interrupts.
  output logic                              intr_done_o,
  output logic                              intr_error_o,

  // DFT-related controls.
  input                                     mbist_en_i,
  input                                     scan_clk_i,
  input                                     scan_rst_ni,
  input prim_mubi_pkg::mubi4_t              scanmode_i
);

  localparam int unsigned DataWidth = top_pkg::TL_DW;

  // Width of buffer address, in bits.
  localparam int unsigned BufWords = 512;
  localparam int unsigned BufAddrW = $clog2(BufWords);

  logic [NumAlerts-1:0] alert_test, alerts;
  // Registers.
  i3c_reg2hw_t reg2hw;
  i3c_hw2reg_t hw2reg;
  // Memory buffer.
  tlul_pkg::tl_h2d_t tl2buf;
  tlul_pkg::tl_d2h_t buf2tl;

  i3c_reg_top #(
    .EnableRacl       (EnableRacl),
    .RaclErrorRsp     (RaclErrorRsp),
    .RaclPolicySelVec (RaclPolicySelVec)
  ) u_reg (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .tl_i             (tl_i),
    .tl_o             (tl_o),

    // Message buffer window.
    .tl_win_o         (tl2buf),
    .tl_win_i         (buf2tl),

    // Registers.
    .reg2hw           (reg2hw),
    .hw2reg           (hw2reg),

    // RACL interface.
    .racl_policies_i  (racl_policies_i),
    .racl_error_o     (racl_error_o),

    // Integrity checking.
    .intg_err_o       (alerts[0])
  );

  // Software interface to the message buffer.
  logic                    swbuf_req;
  logic                    swbuf_gnt;
  logic                    swbuf_we;
  logic     [BufAddrW-1:0] swbuf_addr;
  logic  [DataWidth/8-1:0] swbuf_wmask;
  logic    [DataWidth-1:0] swbuf_wdata;
  logic                    swbuf_rvalid;
  logic              [1:0] swbuf_rerror;
  logic    [DataWidth-1:0] swbuf_rdata;

  logic    [DataWidth-1:0] swbuf_wmask_full;
  always_comb begin
    for (int unsigned b = 0; b < DataWidth/8; b++) begin
      swbuf_wmask[b] = |swbuf_wmask_full[b*8 +: 8];
    end
  end

  tlul_adapter_sram #(
    .SramAw(BufAddrW),
    .ByteAccess(1)
  ) u_tlul2sram (
    .clk_i,
    .rst_ni,

    .tl_i                       (tl2buf),
    .tl_o                       (buf2tl),
    .en_ifetch_i                (prim_mubi_pkg::MuBi4False),
    .req_o                      (swbuf_req),
    .req_type_o                 (),
    .gnt_i                      (swbuf_gnt),
    .we_o                       (swbuf_we),
    .addr_o                     (swbuf_addr),
    .wdata_o                    (swbuf_wdata),
    .wmask_o                    (swbuf_wmask_full),
    .intg_error_o               (),
    .user_rsvd_o                (),
    .rdata_i                    (swbuf_rdata),
    .rvalid_i                   (swbuf_rvalid),
    .rerror_i                   (swbuf_rerror),
    .compound_txn_in_progress_o (),
    .readback_en_i              (prim_mubi_pkg::MuBi4False),
    .readback_error_o           (),
    .wr_collision_i             (1'b0),
    .write_pending_i            (1'b0)
  );

  i3c_core #(
    .BufAddrW  (BufAddrW),
    .DataWidth (DataWidth)
  ) u_core (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),

    .clk_aon_i        (clk_aon_i),
    .rst_aon_ni       (rst_aon_ni),
 
    .ram_cfg_i        (ram_cfg_i),
    .ram_cfg_rsp_o    (ram_cfg_rsp_o),
 
    // Configuration.
    .reg2hw           (reg2hw),
    .hw2reg           (hw2reg),
 
    // Software interface to message buffer.
    .swbuf_req_i      (swbuf_req),
    .swbuf_gnt_o      (swbuf_gnt),
    .swbuf_we_i       (swbuf_we),
    .swbuf_addr_i     (swbuf_addr),
    .swbuf_wmask_i    (swbuf_wmask),
    .swbuf_wdata_i    (swbuf_wdata),
    .swbuf_rvalid_o   (swbuf_rvalid),
    .swbuf_rerror_o   (swbuf_rerror),
    .swbuf_rdata_o    (swbuf_rdata),

    // I3C Controller I/O signaling.
    .ctrl_scl_i       (cio_ctrl_scl_i),
    .ctrl_scl_o       (cio_ctrl_scl_o),
    .ctrl_scl_pp_en_o (cio_ctrl_scl_pp_en_o),
    .ctrl_scl_od_en_o (cio_ctrl_scl_od_en_o),
    .ctrl_sda_i       (cio_ctrl_sda_i),
    .ctrl_sda_o       (cio_ctrl_sda_o),
    .ctrl_sda_pp_en_o (cio_ctrl_sda_pp_en_o),
    .ctrl_sda_od_en_o (cio_ctrl_sda_od_en_o),

    // Pullup enables for open drain intervals.
    .ctrl_scl_pu_en_o (cio_ctrl_scl_pu_en_o),
    .ctrl_sda_pu_en_o (cio_ctrl_sda_pu_en_o),

    // High-keeper enables.
    .scl_hk_en_o      (cio_scl_hk_en_o),
    .sda_hk_en_o      (cio_sda_hk_en_o),

    // I3C Target I/O signaling.
    .targ_scl_i       (cio_targ_scl_i),
    .targ_sda_i       (cio_targ_sda_i),
    .targ_sda_o       (cio_targ_sda_o),
    .targ_sda_pp_en_o (cio_targ_sda_pp_en_o),
    .targ_sda_od_en_o (cio_targ_sda_od_en_o),

    // Chip reset request.
    .chip_rst_req_o   (cio_chip_rst_req_o),

    // Interrupts.
    .intr_done_o      (intr_done_o),
    .intr_error_o     (intr_error_o),

    // DFT-related controls.
    .mbist_en_i       (mbist_en_i),
    .scan_clk_i       (scan_clk_i),
    .scan_rst_ni      (scan_rst_ni),
    .scanmode_i       (scanmode_i)
  );

  // Alerts
  assign alert_test = {
    reg2hw.alert_test.q &
    reg2hw.alert_test.qe
  };

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .SkewCycles(AlertSkewCycles),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[0]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  // TODO: Justify or remove.
  assign lsio_trigger_o = '0;

  // Assert Known for I3C Controller outputs
  `ASSERT_KNOWN(CtrlSCLPPEnKnown_A, cio_ctrl_scl_pp_en_o, clk_i, !rst_ni)
  `ASSERT_KNOWN(CtrlSCLODEnKnown_A, cio_ctrl_scl_od_en_o, clk_i, !rst_ni)
  `ASSERT_KNOWN(CtrlSCLKnown_A, cio_ctrl_scl_o, clk_i, !rst_ni ||
                (!cio_ctrl_scl_pp_en_o & !cio_ctrl_scl_od_en_o))

  `ASSERT_KNOWN(CtrlSDAPPEnKnown_A, cio_ctrl_sda_pp_en_o, clk_i, !rst_ni)
  `ASSERT_KNOWN(CtrlSDAODEnKnown_A, cio_ctrl_sda_od_en_o, clk_i, !rst_ni)
  `ASSERT_KNOWN(CtrlSDAKnown_A, cio_ctrl_sda_o, clk_i, !rst_ni ||
                (!cio_ctrl_sda_pp_en_o & !cio_ctrl_sda_od_en_o))

  // Assert Known for I3C Target outputs
  `ASSERT_KNOWN(TargSDAPPEnKnown_A, cio_targ_sda_pp_en_o, clk_i, !rst_ni)
  `ASSERT_KNOWN(TargSDAODEnKnown_A, cio_targ_sda_od_en_o, clk_i, !rst_ni)
  `ASSERT_KNOWN(TargSDAKnown_A, cio_targ_sda_o, clk_i, !rst_ni ||
                (!cio_targ_sda_pp_en_o & !cio_targ_sda_od_en_o))

  // Assert Known for alerts
  `ASSERT_KNOWN(AlertsKnown_A, alert_tx_o)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])

  // Assert Known for interrupts
  `ASSERT_KNOWN(DoneKnown_A, intr_done_o)
  `ASSERT_KNOWN(ErrorKnown_A, intr_error_o)

endmodule
