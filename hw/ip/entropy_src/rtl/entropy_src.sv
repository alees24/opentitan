// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Description: entropy_src top level wrapper file

`include "prim_assert.sv"


module entropy_src
  import entropy_src_pkg::*;
  import entropy_src_reg_pkg::*;
  import prim_mubi_pkg::mubi8_t;
#(
  parameter logic [NumAlerts-1:0] AlertAsyncOn    = {NumAlerts{1'b1}},
  // Number of cycles a differential skew is tolerated on the alert signal
  parameter int unsigned AlertSkewCycles          = 1,
  parameter int RngBusWidth                       = 4,
  parameter int RngBusBitSelWidth                 = 2,
  parameter int HealthTestWindowWidth             = 18,
  parameter int EsFifoDepth                       = 3,
  parameter int DistrFifoDepth                    = 2,
  parameter bit Stub                              = 1'b0
) (
  input logic clk_i,
  input logic rst_ni,

  // Bus Interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // OTP Interface
  // SEC_CM: INTERSIG.MUBI
  input  mubi8_t otp_en_entropy_src_fw_read_i,
  // SEC_CM: INTERSIG.MUBI
  input  mubi8_t otp_en_entropy_src_fw_over_i,

  // RNG Interface
  output logic rng_fips_o,

  // Entropy Interface
  input  entropy_src_hw_if_req_t entropy_src_hw_if_i,
  output entropy_src_hw_if_rsp_t entropy_src_hw_if_o,

  // RNG Interface
  output logic                   entropy_src_rng_enable_o,
  input  logic                   entropy_src_rng_valid_i,
  input  logic [RngBusWidth-1:0] entropy_src_rng_bits_i,

  // CSRNG Interface
  output cs_aes_halt_req_t cs_aes_halt_o,
  input  cs_aes_halt_rsp_t cs_aes_halt_i,

  // External Health Test Interface
  output logic                             entropy_src_xht_valid_o,
  output logic [RngBusWidth-1:0]           entropy_src_xht_bits_o,
  output logic [RngBusBitSelWidth-1:0]     entropy_src_xht_bit_sel_o,
  output logic [HealthTestWindowWidth-1:0] entropy_src_xht_health_test_window_o,
  output entropy_src_xht_meta_req_t        entropy_src_xht_meta_o,
  input  entropy_src_xht_meta_rsp_t        entropy_src_xht_meta_i,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // Interrupts
  output logic    intr_es_entropy_valid_o,
  output logic    intr_es_health_test_failed_o,
  output logic    intr_es_observe_fifo_ready_o,
  output logic    intr_es_fatal_err_o
);

  // For a data width of N, there are 2^N buckets in the bucket health test, each with its own
  // counter. To make this health test scale reasonably well with `RngBusWidth`, we limit the bucket
  // health test to 4 bit. If `RngBusWidth` is larger than 4, the bucket health test gets
  // instantiated multiple times, once per 4 bit.
  localparam int BucketHtDataWidth = bucket_ht_data_width(RngBusWidth);
  localparam int NumBuckets = 2**BucketHtDataWidth; // bucket health test bin count
  localparam int unsigned NumBucketHtInst = num_bucket_ht_inst(RngBusWidth);

  // Register interface supports up to 256-bit wide noise source bus widths. Ensure the that the
  // parameter does not exceed that limit.
  `ASSERT_INIT(RngBusBitWidthMaxValue_A, RngBusWidth <= 256)
  // The IP needs to define both RngBusWidth, RngBusBitSelWidth, and HealthTestWindowWidth, although
  // the last two are derived from the first. This is needed to allow topgen to pick up the widths
  // for interfaces IP-level interfaces based on these parameters.
  `ASSERT_INIT(RngBusBitSelWidthSameAsComputed_A,
               RngBusBitSelWidth == prim_util_pkg::vbits(RngBusWidth))
  `ASSERT_INIT(HealthTestWindowWidthSameAsComputed_A,
               HealthTestWindowWidth == 16 + prim_util_pkg::vbits(RngBusWidth))

  // common signals
  entropy_src_hw2reg_t hw2reg;
  entropy_src_reg2hw_t reg2hw;
  logic [NumAlerts-1:0] alert_test;
  logic [NumAlerts-1:0] alert;

  // core signals
  logic core_rst_n;
  entropy_src_hw2reg_t core_hw2reg;
  entropy_src_hw_if_rsp_t core_entropy_hw_if;
  logic core_rng_enable;
  cs_aes_halt_req_t core_aes_halt;
  entropy_src_xht_meta_req_t core_xht_meta;
  logic core_xht_valid;
  logic [RngBusWidth-1:0] core_xht_bits;
  logic [RngBusBitSelWidth-1:0] core_xht_bit_sel;
  logic [HealthTestWindowWidth-1:0] core_xht_health_test_window;
  logic core_intr_es_entropy_valid;
  logic core_intr_es_health_test_failed;
  logic core_intr_es_observe_fifo_ready;
  logic core_intr_es_fatal_err;
  logic [NumAlerts-1:0] core_alert_test;
  logic [NumAlerts-1:0] core_alert;

  //stub signals
  localparam int StubLfsrWidth = 64;
  localparam int Copies = CSRNG_BUS_WIDTH / StubLfsrWidth;
  entropy_src_hw2reg_t stub_hw2reg;
  entropy_src_hw_if_rsp_t stub_entropy_hw_if;
  logic stub_es_valid;
  logic [NumAlerts-1:0] stub_alert_test;
  logic [NumAlerts-1:0] stub_alert;
  logic [StubLfsrWidth-1:0] stub_lfsr_value;

  ///////////////////////////
  // Selecting between core and stub
  ///////////////////////////

  assign hw2reg                               = Stub ? stub_hw2reg        : core_hw2reg;
  assign core_rst_n                           = Stub ? '0                 : rst_ni;
  assign entropy_src_hw_if_o                  = Stub ? stub_entropy_hw_if : core_entropy_hw_if;
  assign entropy_src_rng_enable_o             = Stub ? '1                 : core_rng_enable;
  assign cs_aes_halt_o                        = Stub ? '0                 : core_aes_halt;
  assign entropy_src_xht_valid_o              = Stub ? '0                 : core_xht_valid;
  assign entropy_src_xht_bits_o               = Stub ? '0                 : core_xht_bits;
  assign entropy_src_xht_bit_sel_o            = Stub ? '0                 : core_xht_bit_sel;
  assign entropy_src_xht_health_test_window_o = Stub ? '0                 :
                                                       core_xht_health_test_window;
  assign entropy_src_xht_meta_o               = Stub ? '0                 : core_xht_meta;
  assign intr_es_entropy_valid_o              = Stub ? stub_es_valid      :
                                                       core_intr_es_entropy_valid;
  assign intr_es_health_test_failed_o         = Stub ? '0                 :
                                                       core_intr_es_health_test_failed;
  assign intr_es_observe_fifo_ready_o         = Stub ? '0                 :
                                                       core_intr_es_observe_fifo_ready;
  assign intr_es_fatal_err_o                  = Stub ? '0                 : core_intr_es_fatal_err;
  assign alert_test                           = Stub ? stub_alert_test    : core_alert_test;
  assign alert                                = Stub ? stub_alert         : core_alert;

  ///////////////////////////
  // core entropy operation
  ///////////////////////////

  logic [NumAlerts-1:0] intg_err_alert;
  assign intg_err_alert[0] = 1'b0;

  // SEC_CM: CONFIG.REGWEN
  // SEC_CM: TILE_LINK.BUS.INTEGRITY

  entropy_src_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg(hw2reg),
    .intg_err_o(intg_err_alert[1]) // Assign this alert to the fatal alert index.
  );

  entropy_src_core #(
    .RngBusWidth(RngBusWidth),
    .RngBusBitSelWidth(RngBusBitSelWidth),
    .HealthTestWindowWidth(HealthTestWindowWidth),
    .EsFifoDepth(EsFifoDepth),
    .DistrFifoDepth(DistrFifoDepth),
    .BucketHtDataWidth(BucketHtDataWidth),
    .NumBucketHtInst(NumBucketHtInst)
  ) u_entropy_src_core (
    .clk_i,
    .rst_ni(core_rst_n),
    .reg2hw,
    .hw2reg(core_hw2reg),

    .otp_en_entropy_src_fw_read_i(otp_en_entropy_src_fw_read_i),
    .otp_en_entropy_src_fw_over_i(otp_en_entropy_src_fw_over_i),
    .rng_fips_o,

    .entropy_src_hw_if_o(core_entropy_hw_if),
    .entropy_src_hw_if_i,

    .entropy_src_xht_valid_o(core_xht_valid),
    .entropy_src_xht_bits_o(core_xht_bits),
    .entropy_src_xht_bit_sel_o(core_xht_bit_sel),
    .entropy_src_xht_meta_o(core_xht_meta),
    .entropy_src_xht_health_test_window_o(core_xht_health_test_window),
    .entropy_src_xht_meta_i,

    .entropy_src_rng_enable_o(core_rng_enable),
    .entropy_src_rng_valid_i,
    .entropy_src_rng_bits_i,

    .cs_aes_halt_o(core_aes_halt),
    .cs_aes_halt_i,

    .recov_alert_o(core_alert[0]),
    .fatal_alert_o(core_alert[1]),

    .recov_alert_test_o(core_alert_test[0]),
    .fatal_alert_test_o(core_alert_test[1]),

    .intr_es_entropy_valid_o(core_intr_es_entropy_valid),
    .intr_es_health_test_failed_o(core_intr_es_health_test_failed),
    .intr_es_observe_fifo_ready_o(core_intr_es_observe_fifo_ready),
    .intr_es_fatal_err_o(core_intr_es_fatal_err)
  );

  ///////////////////////////
  // stub entropy operation
  ///////////////////////////

  assign stub_alert = '0;
  assign stub_alert_test = '0;
  assign stub_entropy_hw_if = '{
    es_ack:  '1,
    es_bits:  {Copies{stub_lfsr_value}},
    es_fips: '1
  };
  // once enabled, stub entropy is always available

  import prim_mubi_pkg::mubi4_t;
  import prim_mubi_pkg::mubi4_test_true_strict;

  mubi4_t mubi_module_en;
  assign mubi_module_en  = mubi4_t'(reg2hw.module_enable.q);
  assign stub_es_valid = mubi4_test_true_strict(mubi_module_en);

  if (Stub) begin : gen_stub_entropy_src
    prim_lfsr #(
      .LfsrDw(StubLfsrWidth),
      .StateOutDw(StubLfsrWidth)
    ) u_prim_lfsr (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .seed_en_i      ('0),
      .seed_i         ('0),
      .lfsr_en_i      (stub_es_valid),
      .entropy_i      ('0),
      .state_o        (stub_lfsr_value)
    );

    // hardwire hw2reg inputs
    always_comb begin
      stub_hw2reg = '0;

      // as long as enable is 1, do not allow registers to be written
      stub_hw2reg.fw_ov_rd_data.d = stub_lfsr_value[31:0];
      stub_hw2reg.entropy_data.d = stub_lfsr_value[31:0];
      stub_hw2reg.debug_status.main_sm_idle.d = 1'b1;
      stub_hw2reg.main_sm_state.d = entropy_src_main_sm_pkg::Idle;

      stub_hw2reg.intr_state.es_entropy_valid.de = stub_es_valid;
      stub_hw2reg.intr_state.es_entropy_valid.d = 1'b1;

    end
  end else begin : gen_stub_tieoff
    assign stub_hw2reg = '0;
    assign stub_lfsr_value = '0;
  end

  ///////////////////////////
  // Alert generation
  ///////////////////////////
  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .SkewCycles(AlertSkewCycles),
      .IsFatal(i)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i]                 ),
      .alert_req_i   ( alert[i] || intg_err_alert[i] ),
      .alert_ack_o   (                               ),
      .alert_state_o (                               ),
      .alert_rx_i    ( alert_rx_i[i]                 ),
      .alert_tx_o    ( alert_tx_o[i]                 )
    );
  end

  // Outputs should have a known value after reset
  `ASSERT_KNOWN(TlDValidKnownO_A, tl_o.d_valid)
  `ASSERT_KNOWN(TlAReadyKnownO_A, tl_o.a_ready)

  // Entropy Interface
  `ASSERT_KNOWN(EsHwIfEsAckKnownO_A, entropy_src_hw_if_o.es_ack)
  `ASSERT_KNOWN_IF(EsHwIfEsBitsKnownO_A, entropy_src_hw_if_o.es_bits,
      entropy_src_hw_if_o.es_ack)
  `ASSERT_KNOWN_IF(EsHwIfEsFipsKnownO_A, entropy_src_hw_if_o.es_fips,
      entropy_src_hw_if_o.es_ack)

  // RNG Interface
  `ASSERT_KNOWN(EsRngEnableKnownO_A, entropy_src_rng_enable_o)

  // External Health Test Interface
  `ASSERT_KNOWN_IF(EsXhtEntropyBitKnownO_A, entropy_src_xht_bits_o, entropy_src_xht_valid_o)
  `ASSERT_KNOWN(EsXhtEntropyBitValidKnownO_A, entropy_src_xht_valid_o)
  `ASSERT_KNOWN(EsXhtClearKnownO_A, entropy_src_xht_meta_o.clear)
  `ASSERT_KNOWN(EsXhtActiveKnownO_A, entropy_src_xht_meta_o.active)
  `ASSERT_KNOWN(EsXhtThreshHiKnownO_A, entropy_src_xht_meta_o.thresh_hi)
  `ASSERT_KNOWN(EsXhtThreshLoKnownO_A, entropy_src_xht_meta_o.thresh_lo)
  `ASSERT_KNOWN(EsXhtWindowKnownO_A, entropy_src_xht_meta_o.window_wrap_pulse)

  // Alerts
  `ASSERT_KNOWN(AlertTxKnownO_A, alert_tx_o)

  // Interrupts
  `ASSERT_KNOWN(IntrEsEntropyValidKnownO_A, intr_es_entropy_valid_o)
  `ASSERT_KNOWN(IntrEsHealthTestFailedKnownO_A, intr_es_health_test_failed_o)
  `ASSERT_KNOWN(IntrEsFifoErrKnownO_A, intr_es_fatal_err_o)

  // prim_count alerts
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck0_A,
    u_entropy_src_core.u_prim_count_window_cntr,
    alert_tx_o[1])

  for (genvar sh = 0; sh < RngBusWidth; sh = sh+1) begin : gen_bit_cntrs
    `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck1_A,
      u_entropy_src_core.u_entropy_src_adaptp_ht.gen_cntrs[sh].u_prim_count_test_cnt,
      alert_tx_o[1])
  end : gen_bit_cntrs

  for (genvar j = 0; j < NumBucketHtInst; j = j + 1) begin : gen_symbol_match_inst_loop
    for (genvar i = 0; i < NumBuckets; i = i + 1) begin : gen_symbol_match
    `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck_A,
      u_entropy_src_core.gen_health_test[j].u_entropy_src_bucket_ht.gen_symbol_match[i].
      u_prim_count_bin_cntr, alert_tx_o[1])
    end : gen_symbol_match
  end : gen_symbol_match_inst_loop

  for (genvar sh = 0; sh < RngBusWidth; sh = sh+1) begin : gen_pair_cntrs
   `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck_A,
     u_entropy_src_core.u_entropy_src_markov_ht.gen_cntrs[sh].u_prim_count_pair_cntr,
     alert_tx_o[1])
  end : gen_pair_cntrs

  for (genvar sh = 0; sh < RngBusWidth; sh = sh+1) begin : gen_rep_cntrs
   `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck_A,
     u_entropy_src_core.u_entropy_src_repcnt_ht.gen_cntrs[sh].u_prim_count_rep_cntr,
     alert_tx_o[1])
  end : gen_rep_cntrs

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck7_A,
    u_entropy_src_core.u_entropy_src_repcnts_ht.u_prim_count_rep_cntr,
    alert_tx_o[1])

  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(CtrlMainFsmCheck_A,
    u_entropy_src_core.u_entropy_src_main_sm.u_state_regs,
    alert_tx_o[1])

  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(CtrlAckFsmCheck_A,
    u_entropy_src_core.u_entropy_src_ack_sm.u_state_regs,
    alert_tx_o[1])

  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(SHA3FsmCheck_A,
    u_entropy_src_core.u_sha3.u_state_regs, alert_tx_o[1])

  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(KeccakRoundFsmCheck_A,
    u_entropy_src_core.u_sha3.u_keccak.u_state_regs, alert_tx_o[1])

  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(SHA3padFsmCheck_A,
    u_entropy_src_core.u_sha3.u_pad.u_state_regs, alert_tx_o[1])


  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(SentMsgCountCheck_A,
    u_entropy_src_core.u_sha3.u_pad.u_sentmsg_count, alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(RoundCountCheck_A,
    u_entropy_src_core.u_sha3.u_keccak.u_round_count, alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck8_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_repcnt.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck9_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_repcnts.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck10_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_adaptp_hi.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck11_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_adaptp_lo.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck12_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_bucket.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck13_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_markov_hi.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck14_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_markov_lo.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck15_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_extht_hi.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck16_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_extht_lo.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck17_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_any_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck18_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_repcnt_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck19_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_repcnts_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck20_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_adaptp_hi_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck21_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_adaptp_lo_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck22_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_bucket_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck23_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_markov_hi_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck24_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_markov_lo_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck25_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_extht_hi_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(CntAlertCheck26_A,
    u_entropy_src_core.u_entropy_src_cntr_reg_extht_lo_alert_fails.u_prim_count_cntr_reg,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EsrngFifoWptrCheck_A,
    u_entropy_src_core.u_prim_fifo_sync_esrng.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
    alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EsrngFifoRptrCheck_A,
    u_entropy_src_core.u_prim_fifo_sync_esrng.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(DistrFifoWptrCheck_A,
    u_entropy_src_core.u_prim_fifo_sync_distr.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
    alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(DistrFifoRptrCheck_A,
    u_entropy_src_core.u_prim_fifo_sync_distr.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(ObserveFifoWptrCheck_A,
    u_entropy_src_core.u_prim_fifo_sync_observe.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
    alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(ObserveFifoRptrCheck_A,
    u_entropy_src_core.u_prim_fifo_sync_observe.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
    alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EsfinalFifoWptrCheck_A,
    u_entropy_src_core.u_prim_fifo_sync_esfinal.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
    alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EsfinalFifoRptrCheck_A,
    u_entropy_src_core.u_prim_fifo_sync_esfinal.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
    alert_tx_o[1])

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[1])

endmodule
