// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ------------------- W A R N I N G: A U T O - G E N E R A T E D   C O D E !! -------------------//
// PLEASE DO NOT HAND-EDIT THIS FILE. IT HAS BEEN AUTO-GENERATED WITH THE FOLLOWING COMMAND:
//
// util/topgen.py -t hw/top_darjeeling/data/top_darjeeling.hjson \
//                -o hw/top_darjeeling/ \
//                --rnd_cnst_seed \
//                1017106219537032642877583828875051302543807092889754935647094601236425074047

`include "prim_assert.sv"

module top_darjeeling #(
  // Manually defined parameters

  // Auto-inferred parameters
  // parameters for uart0
  // parameters for gpio
  parameter bit GpioGpioAsyncOn = 1,
  parameter bit GpioGpioAsHwStrapsEn = 1,
  // parameters for spi_device
  parameter spi_device_pkg::sram_type_e SpiDeviceSramType = spi_device_pkg::SramType1r1w,
  // parameters for i2c0
  parameter int I2c0InputDelayCycles = 0,
  // parameters for rv_timer
  // parameters for otp_ctrl
  // parameters for otp_macro
  parameter OtpMacroMemInitFile = "",
  // parameters for lc_ctrl
  parameter bit SecLcCtrlVolatileRawUnlockEn = top_pkg::SecVolatileRawUnlockEn,
  parameter bit LcCtrlUseDmiInterface = 1,
  parameter logic [15:0] LcCtrlSiliconCreatorId = 16'h 4002,
  parameter logic [15:0] LcCtrlProductId = 16'h 4000,
  parameter logic [7:0] LcCtrlRevisionId = 8'h 01,
  parameter logic [31:0] LcCtrlIdcodeValue = 32'h00000001,
  // parameters for alert_handler
  parameter int AlertHandlerEscNumSeverities = 4,
  parameter int AlertHandlerEscPingCountWidth = 16,
  // parameters for spi_host0
  // parameters for pwrmgr_aon
  // parameters for rstmgr_aon
  parameter bit SecRstmgrAonCheck = 1'b1,
  parameter int SecRstmgrAonMaxSyncDelay = 2,
  // parameters for clkmgr_aon
  // parameters for pinmux_aon
  parameter pinmux_pkg::target_cfg_t PinmuxAonTargetCfg = pinmux_pkg::DefaultTargetCfg,
  // parameters for aon_timer_aon
  // parameters for soc_proxy
  // parameters for sram_ctrl_ret_aon
  parameter int SramCtrlRetAonInstSize = 4096,
  parameter int SramCtrlRetAonNumRamInst = 1,
  parameter bit SramCtrlRetAonInstrExec = 0,
  parameter int SramCtrlRetAonNumPrinceRoundsHalf = 3,
  parameter bit SramCtrlRetAonEccCorrection = 0,
  // parameters for rv_dm
  parameter logic [31:0] RvDmIdcodeValue = 32'h 0000_0001,
  parameter bit RvDmUseDmiInterface = 1,
  parameter bit SecRvDmVolatileRawUnlockEn = top_pkg::SecVolatileRawUnlockEn,
  parameter logic [tlul_pkg::RsvdWidth-1:0] RvDmTlulHostUserRsvdBits = '0,
  // parameters for rv_plic
  // parameters for aes
  parameter bit SecAesMasking = 1,
  parameter aes_pkg::sbox_impl_e SecAesSBoxImpl = aes_pkg::SBoxImplDom,
  parameter int unsigned SecAesStartTriggerDelay = 0,
  parameter bit SecAesAllowForcingMasks = 1'b0,
  parameter bit SecAesSkipPRNGReseeding = 1'b0,
  // parameters for hmac
  // parameters for kmac
  parameter bit KmacEnMasking = 1,
  parameter bit KmacSwKeyMasked = 0,
  parameter int SecKmacCmdDelay = 0,
  parameter bit SecKmacIdleAcceptSwMsg = 0,
  parameter int KmacNumAppIntf = 4,
  parameter kmac_pkg::app_config_t KmacAppCfg[KmacNumAppIntf] =
      '{kmac_pkg::AppCfgKeyMgr,
        kmac_pkg::AppCfgLcCtrl,
        kmac_pkg::AppCfgRomCtrl,
        kmac_pkg::AppCfgRomCtrl},
  // parameters for otbn
  parameter bit OtbnStub = 0,
  parameter otbn_pkg::regfile_e OtbnRegFile = otbn_pkg::RegFileFF,
  parameter bit SecOtbnMuteUrnd = 0,
  parameter bit SecOtbnSkipUrndReseedAtStart = 0,
  // parameters for keymgr_dpe
  parameter bit KeymgrDpeKmacEnMasking = 1,
  // parameters for csrng
  parameter aes_pkg::sbox_impl_e CsrngSBoxImpl = aes_pkg::SBoxImplCanright,
  // parameters for entropy_src
  parameter int EntropySrcRngBusWidth = 16,
  parameter int EntropySrcRngBusBitSelWidth = 4,
  parameter int EntropySrcHealthTestWindowWidth = 20,
  parameter bit EntropySrcStub = 0,
  // parameters for edn0
  // parameters for edn1
  // parameters for sram_ctrl_main
  parameter int SramCtrlMainInstSize = 65536,
  parameter int SramCtrlMainNumRamInst = 1,
  parameter bit SramCtrlMainInstrExec = 1,
  parameter int SramCtrlMainNumPrinceRoundsHalf = 3,
  parameter bit SramCtrlMainEccCorrection = 0,
  // parameters for sram_ctrl_mbox
  parameter int SramCtrlMboxInstSize = 4096,
  parameter int SramCtrlMboxNumRamInst = 1,
  parameter bit SramCtrlMboxInstrExec = 0,
  parameter int SramCtrlMboxNumPrinceRoundsHalf = 3,
  parameter bit SramCtrlMboxEccCorrection = 0,
  // parameters for rom_ctrl0
  parameter RomCtrl0BootRomInitFile = "",
  parameter bit SecRomCtrl0DisableScrambling = 1'b0,
  // parameters for rom_ctrl1
  parameter RomCtrl1BootRomInitFile = "",
  parameter bit SecRomCtrl1DisableScrambling = 1'b0,
  // parameters for dma
  parameter bit DmaEnableDataIntgGen = 1'b1,
  parameter bit DmaEnableRspDataIntgCheck = 1'b1,
  parameter logic [tlul_pkg::RsvdWidth-1:0] DmaTlUserRsvd = '0,
  parameter logic [dma_pkg::SYS_RACL_WIDTH-1:0] DmaSysRacl = '0,
  parameter int unsigned DmaOtAgentId = 0,
  // parameters for mbx0
  // parameters for mbx1
  // parameters for mbx2
  // parameters for mbx3
  // parameters for mbx4
  // parameters for mbx5
  // parameters for mbx6
  // parameters for mbx_jtag
  // parameters for mbx_pcie0
  // parameters for mbx_pcie1
  // parameters for soc_dbg_ctrl
  // parameters for racl_ctrl
  parameter int RaclCtrlNumExternalSubscribingIps = 1,
  // parameters for ac_range_check
  parameter bit AcRangeCheckRangeCheckErrorRsp = 1,
  // parameters for rv_core_ibex
  parameter bit RvCoreIbexPMPEnable = 1,
  parameter int unsigned RvCoreIbexPMPGranularity = 0,
  parameter int unsigned RvCoreIbexPMPNumRegions = 16,
  parameter int unsigned RvCoreIbexMHPMCounterNum = 10,
  parameter int unsigned RvCoreIbexMHPMCounterWidth = 32,
  parameter ibex_pkg::pmp_cfg_t RvCoreIbexPMPRstCfg[16] = ibex_pmp_reset_pkg::PmpCfgRst,
  parameter logic [33:0] RvCoreIbexPMPRstAddr[16] = ibex_pmp_reset_pkg::PmpAddrRst,
  parameter ibex_pkg::pmp_mseccfg_t RvCoreIbexPMPRstMsecCfg = ibex_pmp_reset_pkg::PmpMseccfgRst,
  parameter bit RvCoreIbexRV32E = 0,
  parameter ibex_pkg::rv32m_e RvCoreIbexRV32M = ibex_pkg::RV32MSingleCycle,
  parameter ibex_pkg::rv32b_e RvCoreIbexRV32B = ibex_pkg::RV32BOTEarlGrey,
  parameter ibex_pkg::regfile_e RvCoreIbexRegFile = ibex_pkg::RegFileFF,
  parameter bit RvCoreIbexBranchTargetALU = 1,
  parameter bit RvCoreIbexWritebackStage = 1,
  parameter bit RvCoreIbexICache = 1,
  parameter bit RvCoreIbexICacheECC = 1,
  parameter bit RvCoreIbexICacheScramble = 1,
  parameter int unsigned RvCoreIbexICacheNWays = 2,
  parameter bit RvCoreIbexBranchPredictor = 0,
  parameter bit RvCoreIbexDbgTriggerEn = 1,
  parameter int RvCoreIbexDbgHwBreakNum = 4,
  parameter bit RvCoreIbexSecureIbex = 1,
  parameter int unsigned RvCoreIbexDmBaseAddr = tl_main_pkg::ADDR_SPACE_RV_DM__MEM,
  parameter int unsigned RvCoreIbexDmAddrMask = tl_main_pkg::ADDR_MASK_RV_DM__MEM,
  parameter int unsigned RvCoreIbexDmHaltAddr =
      tl_main_pkg::ADDR_SPACE_RV_DM__MEM + dm::HaltAddress[31:0],
  parameter int unsigned RvCoreIbexDmExceptionAddr =
      tl_main_pkg::ADDR_SPACE_RV_DM__MEM + dm::ExceptionAddress[31:0],
  parameter bit RvCoreIbexPipeLine = 1,
  parameter logic [tlul_pkg::RsvdWidth-1:0] RvCoreIbexTlulHostUserRsvdBits = '0,
  parameter logic [31:0] RvCoreIbexCsrMvendorId = '0,
  parameter logic [31:0] RvCoreIbexCsrMimpId = '0
) (
  // Multiplexed I/O
  input        [11:0] mio_in_i,
  output logic [11:0] mio_out_o,
  output logic [11:0] mio_oe_o,
  // Dedicated I/O
  input        [72:0] dio_in_i,
  output logic [72:0] dio_out_o,
  output logic [72:0] dio_oe_o,

  // pad attributes to padring
  output prim_pad_wrapper_pkg::pad_attr_t [pinmux_reg_pkg::NMioPads-1:0] mio_attr_o,
  output prim_pad_wrapper_pkg::pad_attr_t [pinmux_reg_pkg::NDioPads-1:0] dio_attr_o,


  // Inter-module Signal External type
  output lc_ctrl_pkg::lc_tx_t       ast_lc_dft_en_o,
  output lc_ctrl_pkg::lc_tx_t       ast_lc_hw_debug_en_o,
  input  ast_pkg::ast_obs_ctrl_t       obs_ctrl_i,
  input  prim_rom_pkg::rom_cfg_t       rom_ctrl0_cfg_i,
  input  prim_rom_pkg::rom_cfg_t       rom_ctrl1_cfg_i,
  input  prim_ram_1p_pkg::ram_1p_cfg_t       i2c_ram_1p_cfg_i,
  output prim_ram_1p_pkg::ram_1p_cfg_rsp_t       i2c_ram_1p_cfg_rsp_o,
  input  prim_ram_1p_pkg::ram_1p_cfg_t [SramCtrlRetAonNumRamInst-1:0] sram_ctrl_ret_aon_ram_1p_cfg_i,
  output prim_ram_1p_pkg::ram_1p_cfg_rsp_t [SramCtrlRetAonNumRamInst-1:0] sram_ctrl_ret_aon_ram_1p_cfg_rsp_o,
  input  prim_ram_1p_pkg::ram_1p_cfg_t [SramCtrlMainNumRamInst-1:0] sram_ctrl_main_ram_1p_cfg_i,
  output prim_ram_1p_pkg::ram_1p_cfg_rsp_t [SramCtrlMainNumRamInst-1:0] sram_ctrl_main_ram_1p_cfg_rsp_o,
  input  prim_ram_1p_pkg::ram_1p_cfg_t [SramCtrlMboxNumRamInst-1:0] sram_ctrl_mbox_ram_1p_cfg_i,
  output prim_ram_1p_pkg::ram_1p_cfg_rsp_t [SramCtrlMboxNumRamInst-1:0] sram_ctrl_mbox_ram_1p_cfg_rsp_o,
  input  prim_ram_1p_pkg::ram_1p_cfg_t       otbn_imem_ram_1p_cfg_i,
  output prim_ram_1p_pkg::ram_1p_cfg_rsp_t       otbn_imem_ram_1p_cfg_rsp_o,
  input  prim_ram_1p_pkg::ram_1p_cfg_t       otbn_dmem_ram_1p_cfg_i,
  output prim_ram_1p_pkg::ram_1p_cfg_rsp_t       otbn_dmem_ram_1p_cfg_rsp_o,
  input  prim_ram_1p_pkg::ram_1p_cfg_t       rv_core_ibex_icache_tag_ram_1p_cfg_i,
  output prim_ram_1p_pkg::ram_1p_cfg_rsp_t [RvCoreIbexICacheNWays-1:0] rv_core_ibex_icache_tag_ram_1p_cfg_rsp_o,
  input  prim_ram_1p_pkg::ram_1p_cfg_t       rv_core_ibex_icache_data_ram_1p_cfg_i,
  output prim_ram_1p_pkg::ram_1p_cfg_rsp_t [RvCoreIbexICacheNWays-1:0] rv_core_ibex_icache_data_ram_1p_cfg_rsp_o,
  input  prim_ram_2p_pkg::ram_2p_cfg_t       spi_device_ram_2p_cfg_sys2spi_i,
  output prim_ram_2p_pkg::ram_2p_cfg_rsp_t       spi_device_ram_2p_cfg_rsp_sys2spi_o,
  output prim_ram_2p_pkg::ram_2p_cfg_rsp_t       spi_device_ram_2p_cfg_rsp_spi2sys_o,
  input  prim_ram_2p_pkg::ram_2p_cfg_t       spi_device_ram_2p_cfg_spi2sys_i,
  output pwrmgr_pkg::pwr_boot_status_t       pwrmgr_boot_status_o,
  output prim_mubi_pkg::mubi4_t       clk_main_jitter_en_o,
  output prim_mubi_pkg::mubi4_t       io_clk_byp_req_o,
  input  prim_mubi_pkg::mubi4_t       io_clk_byp_ack_i,
  output prim_mubi_pkg::mubi4_t       all_clk_byp_req_o,
  input  prim_mubi_pkg::mubi4_t       all_clk_byp_ack_i,
  output prim_mubi_pkg::mubi4_t       hi_speed_sel_o,
  input  prim_mubi_pkg::mubi4_t       div_step_down_req_i,
  input  prim_mubi_pkg::mubi4_t       calib_rdy_i,
  output dma_pkg::sys_req_t       dma_sys_req_o,
  input  dma_pkg::sys_rsp_t       dma_sys_rsp_i,
  output logic       es_rng_enable_o,
  input  logic       es_rng_valid_i,
  input  logic [EntropySrcRngBusWidth-1:0] es_rng_bit_i,
  output logic       es_rng_fips_o,
  input  tlul_pkg::tl_h2d_t       mbx_tl_req_i,
  output tlul_pkg::tl_d2h_t       mbx_tl_rsp_o,
  output logic       mbx0_doe_intr_o,
  output logic       mbx0_doe_intr_en_o,
  output logic       mbx0_doe_intr_support_o,
  output logic       mbx0_doe_async_msg_support_o,
  output logic       mbx1_doe_intr_o,
  output logic       mbx1_doe_intr_en_o,
  output logic       mbx1_doe_intr_support_o,
  output logic       mbx1_doe_async_msg_support_o,
  output logic       mbx2_doe_intr_o,
  output logic       mbx2_doe_intr_en_o,
  output logic       mbx2_doe_intr_support_o,
  output logic       mbx2_doe_async_msg_support_o,
  output logic       mbx3_doe_intr_o,
  output logic       mbx3_doe_intr_en_o,
  output logic       mbx3_doe_intr_support_o,
  output logic       mbx3_doe_async_msg_support_o,
  output logic       mbx4_doe_intr_o,
  output logic       mbx4_doe_intr_en_o,
  output logic       mbx4_doe_intr_support_o,
  output logic       mbx4_doe_async_msg_support_o,
  output logic       mbx5_doe_intr_o,
  output logic       mbx5_doe_intr_en_o,
  output logic       mbx5_doe_intr_support_o,
  output logic       mbx5_doe_async_msg_support_o,
  output logic       mbx6_doe_intr_o,
  output logic       mbx6_doe_intr_en_o,
  output logic       mbx6_doe_intr_support_o,
  output logic       mbx6_doe_async_msg_support_o,
  output logic       mbx_jtag_doe_intr_o,
  output logic       mbx_jtag_doe_intr_en_o,
  output logic       mbx_jtag_doe_intr_support_o,
  output logic       mbx_jtag_doe_async_msg_support_o,
  output logic       mbx_pcie0_doe_intr_o,
  output logic       mbx_pcie0_doe_intr_en_o,
  output logic       mbx_pcie0_doe_intr_support_o,
  output logic       mbx_pcie0_doe_async_msg_support_o,
  output logic       mbx_pcie1_doe_intr_o,
  output logic       mbx_pcie1_doe_intr_en_o,
  output logic       mbx_pcie1_doe_intr_support_o,
  output logic       mbx_pcie1_doe_async_msg_support_o,
  input  tlul_pkg::tl_h2d_t       dbg_tl_req_i,
  output tlul_pkg::tl_d2h_t       dbg_tl_rsp_o,
  input  rv_dm_pkg::next_dm_addr_t       rv_dm_next_dm_addr_i,
  output tlul_pkg::tl_h2d_t       ast_tl_req_o,
  input  tlul_pkg::tl_d2h_t       ast_tl_rsp_i,
  output pwrmgr_pkg::pwr_ast_req_t       pwrmgr_ast_req_o,
  input  pwrmgr_pkg::pwr_ast_rsp_t       pwrmgr_ast_rsp_i,
  output otp_macro_pkg::pwr_seq_t       otp_macro_pwr_seq_o,
  input  otp_macro_pkg::pwr_seq_t       otp_macro_pwr_seq_h_i,
  inout         otp_ext_voltage_h_io,
  output logic [7:0] otp_obs_o,
  input  otp_macro_pkg::otp_cfg_t       otp_cfg_i,
  output otp_macro_pkg::otp_cfg_rsp_t       otp_cfg_rsp_o,
  input  logic [1:0] por_n_i,
  input  logic [31:0] fpga_info_i,
  input  tlul_pkg::tl_h2d_t       ctn_misc_tl_h2d_i,
  output tlul_pkg::tl_d2h_t       ctn_misc_tl_d2h_o,
  input  soc_proxy_pkg::soc_alert_req_t [23:0] soc_fatal_alert_req_i,
  output soc_proxy_pkg::soc_alert_rsp_t [23:0] soc_fatal_alert_rsp_o,
  input  soc_proxy_pkg::soc_alert_req_t [3:0] soc_recov_alert_req_i,
  output soc_proxy_pkg::soc_alert_rsp_t [3:0] soc_recov_alert_rsp_o,
  input  logic       soc_wkup_async_i,
  input  logic       soc_rst_req_async_i,
  input  logic [31:0] soc_intr_async_i,
  input  logic [7:0] soc_lsio_trigger_i,
  output logic [15:0] soc_gpi_async_o,
  input  logic [15:0] soc_gpo_async_i,
  input  logic [3:0] integrator_id_i,
  output logic       sck_monitor_o,
  output soc_dbg_ctrl_pkg::soc_dbg_policy_t       soc_dbg_policy_bus_o,
  input  logic       debug_halt_cpu_boot_i,
  output top_racl_pkg::racl_policy_vec_t       racl_policies_o,
  input  top_racl_pkg::racl_error_log_t [RaclCtrlNumExternalSubscribingIps-1:0] racl_error_i,
  input  prim_mubi_pkg::mubi8_t       ac_range_check_overwrite_i,
  output tlul_pkg::tl_h2d_t       ctn_tl_h2d_o,
  input  tlul_pkg::tl_d2h_t       ctn_tl_d2h_i,


  // All externally supplied clocks
  input clk_main_i,
  input clk_io_i,
  input clk_aon_i,

  // All clocks forwarded to ast
  output clkmgr_pkg::clkmgr_out_t clks_ast_o,
  output rstmgr_pkg::rstmgr_out_t rsts_ast_o,

  input                      scan_rst_ni, // reset used for test mode
  input                      scan_en_i,
  input prim_mubi_pkg::mubi4_t scanmode_i   // lc_ctrl_pkg::On for Scan
);

  import tlul_pkg::*;
  import top_pkg::*;
  import tl_main_pkg::*;
  import top_darjeeling_pkg::*;
  // Compile-time random constants
  import top_darjeeling_rnd_cnst_pkg::*;
  import top_darjeeling_racl_pkg::*;

  // Local Parameters
  // local parameters for lc_ctrl
  localparam int LcCtrlNumRmaAckSigs = 1;
  // local parameters for spi_host0
  localparam int SpiHost0NumCS = 1;
  // local parameters for sram_ctrl_ret_aon
  localparam int SramCtrlRetAonOutstanding = 2;
  // local parameters for entropy_src
  localparam int EntropySrcEsFifoDepth = 3;
  localparam int unsigned EntropySrcDistrFifoDepth = 26;
  // local parameters for sram_ctrl_main
  localparam int SramCtrlMainOutstanding = 2;
  // local parameters for sram_ctrl_mbox
  localparam int SramCtrlMboxOutstanding = 2;
  // local parameters for racl_ctrl
  localparam int RaclCtrlNumSubscribingIps = 11;

  // Signals
  logic [3:0] mio_p2d;
  logic [4:0] mio_d2p;
  logic [4:0] mio_en_d2p;
  logic [72:0] dio_p2d;
  logic [72:0] dio_d2p;
  logic [72:0] dio_en_d2p;
  // uart0
  logic        cio_uart0_rx_p2d;
  logic        cio_uart0_tx_d2p;
  logic        cio_uart0_tx_en_d2p;
  // gpio
  logic [31:0] cio_gpio_gpio_p2d;
  logic [31:0] cio_gpio_gpio_d2p;
  logic [31:0] cio_gpio_gpio_en_d2p;
  // spi_device
  logic        cio_spi_device_sck_p2d;
  logic        cio_spi_device_csb_p2d;
  logic        cio_spi_device_tpm_csb_p2d;
  logic [3:0]  cio_spi_device_sd_p2d;
  logic [3:0]  cio_spi_device_sd_d2p;
  logic [3:0]  cio_spi_device_sd_en_d2p;
  // i2c0
  logic        cio_i2c0_sda_p2d;
  logic        cio_i2c0_scl_p2d;
  logic        cio_i2c0_sda_d2p;
  logic        cio_i2c0_sda_en_d2p;
  logic        cio_i2c0_scl_d2p;
  logic        cio_i2c0_scl_en_d2p;
  // rv_timer
  // otp_ctrl
  // otp_macro
  logic [7:0]  cio_otp_macro_test_d2p;
  logic [7:0]  cio_otp_macro_test_en_d2p;
  // lc_ctrl
  // alert_handler
  // spi_host0
  logic [3:0]  cio_spi_host0_sd_p2d;
  logic        cio_spi_host0_sck_d2p;
  logic        cio_spi_host0_sck_en_d2p;
  logic        cio_spi_host0_csb_d2p;
  logic        cio_spi_host0_csb_en_d2p;
  logic [3:0]  cio_spi_host0_sd_d2p;
  logic [3:0]  cio_spi_host0_sd_en_d2p;
  // pwrmgr_aon
  // rstmgr_aon
  // clkmgr_aon
  // pinmux_aon
  // aon_timer_aon
  // soc_proxy
  logic [15:0] cio_soc_proxy_soc_gpi_p2d;
  logic [15:0] cio_soc_proxy_soc_gpo_d2p;
  logic [15:0] cio_soc_proxy_soc_gpo_en_d2p;
  // sram_ctrl_ret_aon
  // rv_dm
  // rv_plic
  // aes
  // hmac
  // kmac
  // otbn
  // keymgr_dpe
  // csrng
  // entropy_src
  // edn0
  // edn1
  // sram_ctrl_main
  // sram_ctrl_mbox
  // rom_ctrl0
  // rom_ctrl1
  // dma
  // mbx0
  // mbx1
  // mbx2
  // mbx3
  // mbx4
  // mbx5
  // mbx6
  // mbx_jtag
  // mbx_pcie0
  // mbx_pcie1
  // soc_dbg_ctrl
  // racl_ctrl
  // ac_range_check
  // rv_core_ibex


  logic [163:0]  intr_vector;
  // Interrupt source list
  logic intr_uart0_tx_watermark;
  logic intr_uart0_rx_watermark;
  logic intr_uart0_tx_done;
  logic intr_uart0_rx_overflow;
  logic intr_uart0_rx_frame_err;
  logic intr_uart0_rx_break_err;
  logic intr_uart0_rx_timeout;
  logic intr_uart0_rx_parity_err;
  logic intr_uart0_tx_empty;
  logic [31:0] intr_gpio_gpio;
  logic intr_spi_device_upload_cmdfifo_not_empty;
  logic intr_spi_device_upload_payload_not_empty;
  logic intr_spi_device_upload_payload_overflow;
  logic intr_spi_device_readbuf_watermark;
  logic intr_spi_device_readbuf_flip;
  logic intr_spi_device_tpm_header_not_empty;
  logic intr_spi_device_tpm_rdfifo_cmd_end;
  logic intr_spi_device_tpm_rdfifo_drop;
  logic intr_i2c0_fmt_threshold;
  logic intr_i2c0_rx_threshold;
  logic intr_i2c0_acq_threshold;
  logic intr_i2c0_rx_overflow;
  logic intr_i2c0_controller_halt;
  logic intr_i2c0_scl_interference;
  logic intr_i2c0_sda_interference;
  logic intr_i2c0_stretch_timeout;
  logic intr_i2c0_sda_unstable;
  logic intr_i2c0_cmd_complete;
  logic intr_i2c0_tx_stretch;
  logic intr_i2c0_tx_threshold;
  logic intr_i2c0_acq_stretch;
  logic intr_i2c0_unexp_stop;
  logic intr_i2c0_host_timeout;
  logic intr_rv_timer_timer_expired_hart0_timer0;
  logic intr_otp_ctrl_otp_operation_done;
  logic intr_otp_ctrl_otp_error;
  logic intr_alert_handler_classa;
  logic intr_alert_handler_classb;
  logic intr_alert_handler_classc;
  logic intr_alert_handler_classd;
  logic intr_spi_host0_error;
  logic intr_spi_host0_spi_event;
  logic intr_pwrmgr_aon_wakeup;
  logic intr_aon_timer_aon_wkup_timer_expired;
  logic intr_aon_timer_aon_wdog_timer_bark;
  logic [31:0] intr_soc_proxy_external;
  logic intr_hmac_hmac_done;
  logic intr_hmac_fifo_empty;
  logic intr_hmac_hmac_err;
  logic intr_kmac_kmac_done;
  logic intr_kmac_fifo_empty;
  logic intr_kmac_kmac_err;
  logic intr_otbn_done;
  logic intr_keymgr_dpe_op_done;
  logic intr_csrng_cs_cmd_req_done;
  logic intr_csrng_cs_entropy_req;
  logic intr_csrng_cs_hw_inst_exc;
  logic intr_csrng_cs_fatal_err;
  logic intr_entropy_src_es_entropy_valid;
  logic intr_entropy_src_es_health_test_failed;
  logic intr_entropy_src_es_observe_fifo_ready;
  logic intr_entropy_src_es_fatal_err;
  logic intr_edn0_edn_cmd_req_done;
  logic intr_edn0_edn_fatal_err;
  logic intr_edn1_edn_cmd_req_done;
  logic intr_edn1_edn_fatal_err;
  logic intr_dma_dma_done;
  logic intr_dma_dma_chunk_done;
  logic intr_dma_dma_error;
  logic intr_mbx0_mbx_ready;
  logic intr_mbx0_mbx_abort;
  logic intr_mbx0_mbx_error;
  logic intr_mbx1_mbx_ready;
  logic intr_mbx1_mbx_abort;
  logic intr_mbx1_mbx_error;
  logic intr_mbx2_mbx_ready;
  logic intr_mbx2_mbx_abort;
  logic intr_mbx2_mbx_error;
  logic intr_mbx3_mbx_ready;
  logic intr_mbx3_mbx_abort;
  logic intr_mbx3_mbx_error;
  logic intr_mbx4_mbx_ready;
  logic intr_mbx4_mbx_abort;
  logic intr_mbx4_mbx_error;
  logic intr_mbx5_mbx_ready;
  logic intr_mbx5_mbx_abort;
  logic intr_mbx5_mbx_error;
  logic intr_mbx6_mbx_ready;
  logic intr_mbx6_mbx_abort;
  logic intr_mbx6_mbx_error;
  logic intr_mbx_jtag_mbx_ready;
  logic intr_mbx_jtag_mbx_abort;
  logic intr_mbx_jtag_mbx_error;
  logic intr_mbx_pcie0_mbx_ready;
  logic intr_mbx_pcie0_mbx_abort;
  logic intr_mbx_pcie0_mbx_error;
  logic intr_mbx_pcie1_mbx_ready;
  logic intr_mbx_pcie1_mbx_abort;
  logic intr_mbx_pcie1_mbx_error;
  logic intr_racl_ctrl_racl_error;
  logic intr_ac_range_check_deny_cnt_reached;

  // Alert list
  prim_alert_pkg::alert_tx_t [alert_handler_pkg::NAlerts-1:0]  alert_tx;
  prim_alert_pkg::alert_rx_t [alert_handler_pkg::NAlerts-1:0]  alert_rx;


  // define inter-module signals
  ast_pkg::ast_obs_ctrl_t       ast_obs_ctrl;
  alert_handler_pkg::alert_crashdump_t       alert_handler_crashdump;
  prim_esc_pkg::esc_rx_t [3:0] alert_handler_esc_rx;
  prim_esc_pkg::esc_tx_t [3:0] alert_handler_esc_tx;
  logic       aon_timer_aon_nmi_wdog_timer_bark;
  csrng_pkg::csrng_req_t [1:0] csrng_csrng_cmd_req;
  csrng_pkg::csrng_rsp_t [1:0] csrng_csrng_cmd_rsp;
  entropy_src_pkg::entropy_src_hw_if_req_t       csrng_entropy_src_hw_if_req;
  entropy_src_pkg::entropy_src_hw_if_rsp_t       csrng_entropy_src_hw_if_rsp;
  entropy_src_pkg::cs_aes_halt_req_t       csrng_cs_aes_halt_req;
  entropy_src_pkg::cs_aes_halt_rsp_t       csrng_cs_aes_halt_rsp;
  otp_ctrl_pkg::sram_otp_key_req_t [3:0] otp_ctrl_sram_otp_key_req;
  otp_ctrl_pkg::sram_otp_key_rsp_t [3:0] otp_ctrl_sram_otp_key_rsp;
  pwrmgr_pkg::pwr_rst_req_t       pwrmgr_aon_pwr_rst_req;
  pwrmgr_pkg::pwr_rst_rsp_t       pwrmgr_aon_pwr_rst_rsp;
  pwrmgr_pkg::pwr_clk_req_t       pwrmgr_aon_pwr_clk_req;
  pwrmgr_pkg::pwr_clk_rsp_t       pwrmgr_aon_pwr_clk_rsp;
  pwrmgr_pkg::pwr_otp_req_t       pwrmgr_aon_pwr_otp_req;
  pwrmgr_pkg::pwr_otp_rsp_t       pwrmgr_aon_pwr_otp_rsp;
  lc_ctrl_pkg::pwr_lc_req_t       pwrmgr_aon_pwr_lc_req;
  lc_ctrl_pkg::pwr_lc_rsp_t       pwrmgr_aon_pwr_lc_rsp;
  logic       pwrmgr_aon_strap;
  logic       pwrmgr_aon_low_power;
  lc_ctrl_pkg::lc_tx_t       pwrmgr_aon_fetch_en;
  rom_ctrl_pkg::pwrmgr_data_t [2:0] pwrmgr_aon_rom_ctrl;
  pwrmgr_pkg::pwr_boot_status_t       pwrmgr_aon_boot_status;
  rom_ctrl_pkg::keymgr_data_t [1:0] keymgr_dpe_rom_digest;
  dma_pkg::lsio_trigger_t       dma_lsio_trigger;
  logic       i2c0_lsio_trigger;
  logic       spi_host0_lsio_trigger;
  logic       uart0_lsio_trigger;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_flash_rma_req;
  lc_ctrl_pkg::lc_tx_t       otbn_lc_rma_ack;
  edn_pkg::edn_req_t [7:0] edn0_edn_req;
  edn_pkg::edn_rsp_t [7:0] edn0_edn_rsp;
  edn_pkg::edn_req_t [7:0] edn1_edn_req;
  edn_pkg::edn_rsp_t [7:0] edn1_edn_rsp;
  otp_ctrl_pkg::otbn_otp_key_req_t       otp_ctrl_otbn_otp_key_req;
  otp_ctrl_pkg::otbn_otp_key_rsp_t       otp_ctrl_otbn_otp_key_rsp;
  otp_ctrl_pkg::otp_keymgr_key_t       otp_ctrl_otp_keymgr_key;
  keymgr_pkg::hw_key_req_t       keymgr_dpe_aes_key;
  keymgr_pkg::hw_key_req_t       keymgr_dpe_kmac_key;
  keymgr_pkg::otbn_key_req_t       keymgr_dpe_otbn_key;
  kmac_pkg::app_req_t [KmacNumAppIntf-1:0] kmac_app_req;
  kmac_pkg::app_rsp_t [KmacNumAppIntf-1:0] kmac_app_rsp;
  logic       kmac_en_masking;
  prim_mubi_pkg::mubi4_t [3:0] clkmgr_aon_idle;
  otp_ctrl_pkg::otp_lc_data_t       otp_ctrl_otp_lc_data;
  otp_ctrl_pkg::lc_otp_program_req_t       lc_ctrl_lc_otp_program_req;
  otp_ctrl_pkg::lc_otp_program_rsp_t       lc_ctrl_lc_otp_program_rsp;
  otp_macro_pkg::otp_test_req_t       lc_ctrl_lc_otp_vendor_test_req;
  otp_macro_pkg::otp_test_rsp_t       lc_ctrl_lc_otp_vendor_test_rsp;
  lc_ctrl_pkg::lc_keymgr_div_t       lc_ctrl_lc_keymgr_div;
  logic       lc_ctrl_strap_en_override;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_raw_test_rma;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_dft_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_hw_debug_clr;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_hw_debug_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_cpu_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_keymgr_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_escalate_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_check_byp_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_clk_byp_req;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_clk_byp_ack;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_creator_seed_sw_rw_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_owner_seed_sw_rw_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_seed_hw_rd_en;
  otp_ctrl_macro_pkg::otp_ctrl_macro_req_t       otp_ctrl_otp_macro_req;
  otp_ctrl_macro_pkg::otp_ctrl_macro_rsp_t       otp_ctrl_otp_macro_rsp;
  logic       rv_plic_msip;
  logic       rv_plic_irq;
  logic       rv_dm_debug_req;
  rv_core_ibex_pkg::cpu_crash_dump_t       rv_core_ibex_crash_dump;
  rv_core_ibex_pkg::cpu_pwrmgr_t       rv_core_ibex_pwrmgr;
  spi_device_pkg::passthrough_req_t       spi_device_passthrough_req;
  spi_device_pkg::passthrough_rsp_t       spi_device_passthrough_rsp;
  logic       rv_dm_ndmreset_req;
  prim_mubi_pkg::mubi4_t       rstmgr_aon_sw_rst_req;
  tlul_pkg::tl_h2d_t       soc_proxy_dma_tl_h2d;
  tlul_pkg::tl_d2h_t       soc_proxy_dma_tl_d2h;
  tlul_pkg::tl_h2d_t       soc_proxy_ctn_tl_h2d;
  tlul_pkg::tl_d2h_t       soc_proxy_ctn_tl_d2h;
  logic [3:0] pwrmgr_aon_wakeups;
  logic [1:0] pwrmgr_aon_rstreqs;
  tlul_pkg::tl_h2d_t       main_tl_rv_core_ibex__corei_req;
  tlul_pkg::tl_d2h_t       main_tl_rv_core_ibex__corei_rsp;
  tlul_pkg::tl_h2d_t       main_tl_rv_core_ibex__cored_req;
  tlul_pkg::tl_d2h_t       main_tl_rv_core_ibex__cored_rsp;
  tlul_pkg::tl_h2d_t       main_tl_rv_dm__sba_req;
  tlul_pkg::tl_d2h_t       main_tl_rv_dm__sba_rsp;
  tlul_pkg::tl_h2d_t       rv_dm_regs_tl_d_req;
  tlul_pkg::tl_d2h_t       rv_dm_regs_tl_d_rsp;
  tlul_pkg::tl_h2d_t       rv_dm_mem_tl_d_req;
  tlul_pkg::tl_d2h_t       rv_dm_mem_tl_d_rsp;
  tlul_pkg::tl_h2d_t       rom_ctrl0_rom_tl_req;
  tlul_pkg::tl_d2h_t       rom_ctrl0_rom_tl_rsp;
  tlul_pkg::tl_h2d_t       rom_ctrl0_regs_tl_req;
  tlul_pkg::tl_d2h_t       rom_ctrl0_regs_tl_rsp;
  tlul_pkg::tl_h2d_t       rom_ctrl1_rom_tl_req;
  tlul_pkg::tl_d2h_t       rom_ctrl1_rom_tl_rsp;
  tlul_pkg::tl_h2d_t       rom_ctrl1_regs_tl_req;
  tlul_pkg::tl_d2h_t       rom_ctrl1_regs_tl_rsp;
  tlul_pkg::tl_h2d_t       main_tl_peri_req;
  tlul_pkg::tl_d2h_t       main_tl_peri_rsp;
  tlul_pkg::tl_h2d_t       soc_proxy_core_tl_req;
  tlul_pkg::tl_d2h_t       soc_proxy_core_tl_rsp;
  tlul_pkg::tl_h2d_t       soc_proxy_ctn_tl_req;
  tlul_pkg::tl_d2h_t       soc_proxy_ctn_tl_rsp;
  tlul_pkg::tl_h2d_t       hmac_tl_req;
  tlul_pkg::tl_d2h_t       hmac_tl_rsp;
  tlul_pkg::tl_h2d_t       kmac_tl_req;
  tlul_pkg::tl_d2h_t       kmac_tl_rsp;
  tlul_pkg::tl_h2d_t       aes_tl_req;
  tlul_pkg::tl_d2h_t       aes_tl_rsp;
  tlul_pkg::tl_h2d_t       entropy_src_tl_req;
  tlul_pkg::tl_d2h_t       entropy_src_tl_rsp;
  tlul_pkg::tl_h2d_t       csrng_tl_req;
  tlul_pkg::tl_d2h_t       csrng_tl_rsp;
  tlul_pkg::tl_h2d_t       edn0_tl_req;
  tlul_pkg::tl_d2h_t       edn0_tl_rsp;
  tlul_pkg::tl_h2d_t       edn1_tl_req;
  tlul_pkg::tl_d2h_t       edn1_tl_rsp;
  tlul_pkg::tl_h2d_t       rv_plic_tl_req;
  tlul_pkg::tl_d2h_t       rv_plic_tl_rsp;
  tlul_pkg::tl_h2d_t       otbn_tl_req;
  tlul_pkg::tl_d2h_t       otbn_tl_rsp;
  tlul_pkg::tl_h2d_t       keymgr_dpe_tl_req;
  tlul_pkg::tl_d2h_t       keymgr_dpe_tl_rsp;
  tlul_pkg::tl_h2d_t       rv_core_ibex_cfg_tl_d_req;
  tlul_pkg::tl_d2h_t       rv_core_ibex_cfg_tl_d_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_main_regs_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_main_regs_tl_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_main_ram_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_main_ram_tl_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_mbox_regs_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_mbox_regs_tl_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_mbox_ram_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_mbox_ram_tl_rsp;
  tlul_pkg::tl_h2d_t       dma_tl_d_req;
  tlul_pkg::tl_d2h_t       dma_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_dma__host_req;
  tlul_pkg::tl_d2h_t       main_tl_dma__host_rsp;
  tlul_pkg::tl_h2d_t       mbx0_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx0_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx0__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx0__sram_rsp;
  tlul_pkg::tl_h2d_t       mbx1_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx1_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx1__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx1__sram_rsp;
  tlul_pkg::tl_h2d_t       mbx2_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx2_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx2__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx2__sram_rsp;
  tlul_pkg::tl_h2d_t       mbx3_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx3_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx3__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx3__sram_rsp;
  tlul_pkg::tl_h2d_t       mbx4_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx4_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx4__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx4__sram_rsp;
  tlul_pkg::tl_h2d_t       mbx5_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx5_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx5__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx5__sram_rsp;
  tlul_pkg::tl_h2d_t       mbx6_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx6_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx6__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx6__sram_rsp;
  tlul_pkg::tl_h2d_t       mbx_jtag_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx_jtag_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx_jtag__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx_jtag__sram_rsp;
  tlul_pkg::tl_h2d_t       mbx_pcie0_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx_pcie0_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx_pcie0__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx_pcie0__sram_rsp;
  tlul_pkg::tl_h2d_t       mbx_pcie1_core_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx_pcie1_core_tl_d_rsp;
  tlul_pkg::tl_h2d_t       main_tl_mbx_pcie1__sram_req;
  tlul_pkg::tl_d2h_t       main_tl_mbx_pcie1__sram_rsp;
  tlul_pkg::tl_h2d_t       uart0_tl_req;
  tlul_pkg::tl_d2h_t       uart0_tl_rsp;
  tlul_pkg::tl_h2d_t       i2c0_tl_req;
  tlul_pkg::tl_d2h_t       i2c0_tl_rsp;
  tlul_pkg::tl_h2d_t       gpio_tl_req;
  tlul_pkg::tl_d2h_t       gpio_tl_rsp;
  tlul_pkg::tl_h2d_t       spi_host0_tl_req;
  tlul_pkg::tl_d2h_t       spi_host0_tl_rsp;
  tlul_pkg::tl_h2d_t       spi_device_tl_req;
  tlul_pkg::tl_d2h_t       spi_device_tl_rsp;
  tlul_pkg::tl_h2d_t       rv_timer_tl_req;
  tlul_pkg::tl_d2h_t       rv_timer_tl_rsp;
  tlul_pkg::tl_h2d_t       pwrmgr_aon_tl_req;
  tlul_pkg::tl_d2h_t       pwrmgr_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       rstmgr_aon_tl_req;
  tlul_pkg::tl_d2h_t       rstmgr_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       clkmgr_aon_tl_req;
  tlul_pkg::tl_d2h_t       clkmgr_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       pinmux_aon_tl_req;
  tlul_pkg::tl_d2h_t       pinmux_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       otp_ctrl_core_tl_req;
  tlul_pkg::tl_d2h_t       otp_ctrl_core_tl_rsp;
  tlul_pkg::tl_h2d_t       otp_macro_prim_tl_req;
  tlul_pkg::tl_d2h_t       otp_macro_prim_tl_rsp;
  tlul_pkg::tl_h2d_t       lc_ctrl_regs_tl_req;
  tlul_pkg::tl_d2h_t       lc_ctrl_regs_tl_rsp;
  tlul_pkg::tl_h2d_t       alert_handler_tl_req;
  tlul_pkg::tl_d2h_t       alert_handler_tl_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_ret_aon_regs_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_ret_aon_regs_tl_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_ret_aon_ram_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_ret_aon_ram_tl_rsp;
  tlul_pkg::tl_h2d_t       aon_timer_aon_tl_req;
  tlul_pkg::tl_d2h_t       aon_timer_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       soc_dbg_ctrl_core_tl_req;
  tlul_pkg::tl_d2h_t       soc_dbg_ctrl_core_tl_rsp;
  tlul_pkg::tl_h2d_t       mbx0_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx0_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       mbx1_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx1_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       mbx2_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx2_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       mbx3_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx3_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       mbx4_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx4_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       mbx5_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx5_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       mbx6_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx6_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       mbx_pcie0_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx_pcie0_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       mbx_pcie1_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx_pcie1_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       racl_ctrl_tl_req;
  tlul_pkg::tl_d2h_t       racl_ctrl_tl_rsp;
  tlul_pkg::tl_h2d_t       ac_range_check_tl_req;
  tlul_pkg::tl_d2h_t       ac_range_check_tl_rsp;
  tlul_pkg::tl_h2d_t       rv_dm_dbg_tl_d_req;
  tlul_pkg::tl_d2h_t       rv_dm_dbg_tl_d_rsp;
  tlul_pkg::tl_h2d_t       mbx_jtag_soc_tl_d_req;
  tlul_pkg::tl_d2h_t       mbx_jtag_soc_tl_d_rsp;
  tlul_pkg::tl_h2d_t       lc_ctrl_dmi_tl_req;
  tlul_pkg::tl_d2h_t       lc_ctrl_dmi_tl_rsp;
  tlul_pkg::tl_h2d_t       soc_dbg_ctrl_jtag_tl_req;
  tlul_pkg::tl_d2h_t       soc_dbg_ctrl_jtag_tl_rsp;
  top_racl_pkg::racl_policy_vec_t       racl_ctrl_racl_policies;
  top_racl_pkg::racl_error_log_t [RaclCtrlNumSubscribingIps-1:0] racl_ctrl_racl_error;
  clkmgr_pkg::clkmgr_out_t       clkmgr_aon_clocks;
  clkmgr_pkg::clkmgr_cg_en_t       clkmgr_aon_cg_en;
  rstmgr_pkg::rstmgr_out_t       rstmgr_aon_resets;
  rstmgr_pkg::rstmgr_rst_en_t       rstmgr_aon_rst_en;
  logic       rv_core_ibex_irq_timer;
  logic [31:0] rv_core_ibex_hart_id;
  logic [31:0] rv_core_ibex_boot_addr;
  otp_ctrl_part_pkg::otp_broadcast_t       otp_ctrl_otp_broadcast;
  prim_mubi_pkg::mubi8_t       csrng_otp_en_csrng_sw_app_read;
  lc_ctrl_state_pkg::soc_dbg_state_t       soc_dbg_ctrl_soc_dbg_state;
  otp_ctrl_pkg::otp_device_id_t       lc_ctrl_otp_device_id;
  otp_ctrl_pkg::otp_manuf_state_t       lc_ctrl_otp_manuf_state;
  otp_ctrl_pkg::otp_device_id_t       keymgr_dpe_otp_device_id;
  prim_mubi_pkg::mubi8_t       sram_ctrl_main_otp_en_sram_ifetch;
  prim_mubi_pkg::mubi8_t       rv_dm_otp_dis_rv_dm_late_debug;

  // define mixed connection to port
  assign ast_lc_dft_en_o = lc_ctrl_lc_dft_en;
  assign ast_lc_hw_debug_en_o = lc_ctrl_lc_hw_debug_en;
  assign ast_obs_ctrl = obs_ctrl_i;
  assign pwrmgr_boot_status_o = pwrmgr_aon_boot_status;
  assign racl_policies_o = racl_ctrl_racl_policies;

  // define partial inter-module tie-off
  edn_pkg::edn_rsp_t unused_edn0_edn_rsp7;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp1;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp2;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp3;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp4;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp5;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp6;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp7;

  // assign partial inter-module tie-off
  assign unused_edn0_edn_rsp7 = edn0_edn_rsp[7];
  assign unused_edn1_edn_rsp1 = edn1_edn_rsp[1];
  assign unused_edn1_edn_rsp2 = edn1_edn_rsp[2];
  assign unused_edn1_edn_rsp3 = edn1_edn_rsp[3];
  assign unused_edn1_edn_rsp4 = edn1_edn_rsp[4];
  assign unused_edn1_edn_rsp5 = edn1_edn_rsp[5];
  assign unused_edn1_edn_rsp6 = edn1_edn_rsp[6];
  assign unused_edn1_edn_rsp7 = edn1_edn_rsp[7];
  assign edn0_edn_req[7] = '0;
  assign edn1_edn_req[1] = '0;
  assign edn1_edn_req[2] = '0;
  assign edn1_edn_req[3] = '0;
  assign edn1_edn_req[4] = '0;
  assign edn1_edn_req[5] = '0;
  assign edn1_edn_req[6] = '0;
  assign edn1_edn_req[7] = '0;


  // OTP HW_CFG Broadcast signals.
  // TODO(#6713): The actual struct breakout and mapping currently needs to
  // be performed by hand.
  assign csrng_otp_en_csrng_sw_app_read =
      otp_ctrl_otp_broadcast.hw_cfg1_data.en_csrng_sw_app_read;
  assign sram_ctrl_main_otp_en_sram_ifetch =
      otp_ctrl_otp_broadcast.hw_cfg1_data.en_sram_ifetch;
  assign lc_ctrl_otp_device_id =
      otp_ctrl_otp_broadcast.hw_cfg0_data.device_id;
  assign soc_dbg_ctrl_soc_dbg_state =
      otp_ctrl_otp_broadcast.hw_cfg1_data.soc_dbg_state;
  assign lc_ctrl_otp_manuf_state =
      otp_ctrl_otp_broadcast.hw_cfg0_data.manuf_state;
  assign keymgr_dpe_otp_device_id =
      otp_ctrl_otp_broadcast.hw_cfg0_data.device_id;

  logic unused_otp_broadcast_bits;
  assign unused_otp_broadcast_bits = ^{
    otp_ctrl_otp_broadcast.valid,
    otp_ctrl_otp_broadcast.hw_cfg0_data.hw_cfg0_digest,
    otp_ctrl_otp_broadcast.hw_cfg1_data.hw_cfg1_digest,
    otp_ctrl_otp_broadcast.hw_cfg1_data.unallocated
  };

  // See #7978 This below is a hack.
  // This is because ast is a comportable-like module that sits outside
  // of top_darjeeling's boundary.
  assign clks_ast_o = clkmgr_aon_clocks;
  assign rsts_ast_o = rstmgr_aon_resets;

  // ibex specific assignments
  // TODO: This should be further automated in the future.
  assign rv_core_ibex_irq_timer = intr_rv_timer_timer_expired_hart0_timer0;
  assign rv_core_ibex_hart_id = '0;

  // Unconditionally disable the late debug feature and enable early debug
  assign rv_dm_otp_dis_rv_dm_late_debug = prim_mubi_pkg::MuBi8True;

  assign rv_core_ibex_boot_addr = ADDR_SPACE_ROM_CTRL0__ROM;

  // Wire up alert handler LPGs
  prim_mubi_pkg::mubi4_t [alert_handler_pkg::NLpg-1:0] lpg_cg_en;
  prim_mubi_pkg::mubi4_t [alert_handler_pkg::NLpg-1:0] lpg_rst_en;


  // peri_lc_io_div4_0
  assign lpg_cg_en[0] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[0] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::Domain0Sel];
  // peri_spi_device_0
  assign lpg_cg_en[1] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[1] = rstmgr_aon_rst_en.spi_device[rstmgr_pkg::Domain0Sel];
  // peri_i2c0_0
  assign lpg_cg_en[2] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[2] = rstmgr_aon_rst_en.i2c0[rstmgr_pkg::Domain0Sel];
  // timers_lc_io_div4_0
  assign lpg_cg_en[3] = clkmgr_aon_cg_en.io_div4_timers;
  assign lpg_rst_en[3] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::Domain0Sel];
  // secure_lc_io_div4_0
  assign lpg_cg_en[4] = clkmgr_aon_cg_en.io_div4_secure;
  assign lpg_rst_en[4] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::Domain0Sel];
  // peri_spi_host0_0
  assign lpg_cg_en[5] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[5] = rstmgr_aon_rst_en.spi_host0[rstmgr_pkg::Domain0Sel];
  // powerup_por_io_div4_Aon
  assign lpg_cg_en[6] = clkmgr_aon_cg_en.io_div4_powerup;
  assign lpg_rst_en[6] = rstmgr_aon_rst_en.por_io_div4[rstmgr_pkg::DomainAonSel];
  // powerup_lc_io_div4_Aon
  assign lpg_cg_en[7] = clkmgr_aon_cg_en.io_div4_powerup;
  assign lpg_rst_en[7] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::DomainAonSel];
  // timers_lc_io_div4_Aon
  assign lpg_cg_en[8] = clkmgr_aon_cg_en.io_div4_timers;
  assign lpg_rst_en[8] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::DomainAonSel];
  // infra_lc_io_div4_0
  assign lpg_cg_en[9] = clkmgr_aon_cg_en.io_div4_infra;
  assign lpg_rst_en[9] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::Domain0Sel];
  // infra_lc_0
  assign lpg_cg_en[10] = clkmgr_aon_cg_en.main_infra;
  assign lpg_rst_en[10] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // infra_lc_io_div4_Aon
  assign lpg_cg_en[11] = clkmgr_aon_cg_en.io_div4_infra;
  assign lpg_rst_en[11] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::DomainAonSel];
  // infra_sys_0
  assign lpg_cg_en[12] = clkmgr_aon_cg_en.main_infra;
  assign lpg_rst_en[12] = rstmgr_aon_rst_en.sys[rstmgr_pkg::Domain0Sel];
  // secure_lc_0
  assign lpg_cg_en[13] = clkmgr_aon_cg_en.main_secure;
  assign lpg_rst_en[13] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // aes_trans_lc_0
  assign lpg_cg_en[14] = clkmgr_aon_cg_en.main_aes;
  assign lpg_rst_en[14] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // hmac_trans_lc_0
  assign lpg_cg_en[15] = clkmgr_aon_cg_en.main_hmac;
  assign lpg_rst_en[15] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // kmac_trans_lc_0
  assign lpg_cg_en[16] = clkmgr_aon_cg_en.main_kmac;
  assign lpg_rst_en[16] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // otbn_trans_lc_0
  assign lpg_cg_en[17] = clkmgr_aon_cg_en.main_otbn;
  assign lpg_rst_en[17] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];


// tie-off unused connections
//VCS coverage off
// pragma coverage off
    prim_mubi_pkg::mubi4_t unused_cg_en_0;
    assign unused_cg_en_0 = clkmgr_aon_cg_en.aon_powerup;
    prim_mubi_pkg::mubi4_t unused_cg_en_1;
    assign unused_cg_en_1 = clkmgr_aon_cg_en.main_powerup;
    prim_mubi_pkg::mubi4_t unused_cg_en_2;
    assign unused_cg_en_2 = clkmgr_aon_cg_en.io_powerup;
    prim_mubi_pkg::mubi4_t unused_cg_en_3;
    assign unused_cg_en_3 = clkmgr_aon_cg_en.io_div2_powerup;
    prim_mubi_pkg::mubi4_t unused_cg_en_4;
    assign unused_cg_en_4 = clkmgr_aon_cg_en.aon_infra;
    prim_mubi_pkg::mubi4_t unused_cg_en_5;
    assign unused_cg_en_5 = clkmgr_aon_cg_en.aon_peri;
    prim_mubi_pkg::mubi4_t unused_cg_en_6;
    assign unused_cg_en_6 = clkmgr_aon_cg_en.aon_timers;
    prim_mubi_pkg::mubi4_t unused_cg_en_7;
    assign unused_cg_en_7 = clkmgr_aon_cg_en.io_div2_peri;
    prim_mubi_pkg::mubi4_t unused_rst_en_0;
    assign unused_rst_en_0 = rstmgr_aon_rst_en.por_aon[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_1;
    assign unused_rst_en_1 = rstmgr_aon_rst_en.por_aon[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_2;
    assign unused_rst_en_2 = rstmgr_aon_rst_en.por[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_3;
    assign unused_rst_en_3 = rstmgr_aon_rst_en.por[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_4;
    assign unused_rst_en_4 = rstmgr_aon_rst_en.por_io[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_5;
    assign unused_rst_en_5 = rstmgr_aon_rst_en.por_io[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_6;
    assign unused_rst_en_6 = rstmgr_aon_rst_en.por_io_div2[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_7;
    assign unused_rst_en_7 = rstmgr_aon_rst_en.por_io_div2[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_8;
    assign unused_rst_en_8 = rstmgr_aon_rst_en.por_io_div4[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_9;
    assign unused_rst_en_9 = rstmgr_aon_rst_en.lc_shadowed[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_10;
    assign unused_rst_en_10 = rstmgr_aon_rst_en.lc[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_11;
    assign unused_rst_en_11 = rstmgr_aon_rst_en.lc_shadowed[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_12;
    assign unused_rst_en_12 = rstmgr_aon_rst_en.lc_aon[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_13;
    assign unused_rst_en_13 = rstmgr_aon_rst_en.lc_aon[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_14;
    assign unused_rst_en_14 = rstmgr_aon_rst_en.lc_io[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_15;
    assign unused_rst_en_15 = rstmgr_aon_rst_en.lc_io[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_16;
    assign unused_rst_en_16 = rstmgr_aon_rst_en.lc_io_div2[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_17;
    assign unused_rst_en_17 = rstmgr_aon_rst_en.lc_io_div2[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_18;
    assign unused_rst_en_18 = rstmgr_aon_rst_en.lc_io_div4_shadowed[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_19;
    assign unused_rst_en_19 = rstmgr_aon_rst_en.lc_io_div4_shadowed[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_20;
    assign unused_rst_en_20 = rstmgr_aon_rst_en.sys[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_21;
    assign unused_rst_en_21 = rstmgr_aon_rst_en.sys_io_div4[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_22;
    assign unused_rst_en_22 = rstmgr_aon_rst_en.sys_io_div4[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_23;
    assign unused_rst_en_23 = rstmgr_aon_rst_en.spi_device[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_24;
    assign unused_rst_en_24 = rstmgr_aon_rst_en.spi_host0[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_25;
    assign unused_rst_en_25 = rstmgr_aon_rst_en.i2c0[rstmgr_pkg::DomainAonSel];
//VCS coverage on
// pragma coverage on

  // Peripheral Instantiation


  uart #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[0:0]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_uart0 (

      // Input
      .cio_rx_i    (cio_uart0_rx_p2d),

      // Output
      .cio_tx_o    (cio_uart0_tx_d2p),
      .cio_tx_en_o (cio_uart0_tx_en_d2p),

      // Interrupt
      .intr_tx_watermark_o  (intr_uart0_tx_watermark),
      .intr_rx_watermark_o  (intr_uart0_rx_watermark),
      .intr_tx_done_o       (intr_uart0_tx_done),
      .intr_rx_overflow_o   (intr_uart0_rx_overflow),
      .intr_rx_frame_err_o  (intr_uart0_rx_frame_err),
      .intr_rx_break_err_o  (intr_uart0_rx_break_err),
      .intr_rx_timeout_o    (intr_uart0_rx_timeout),
      .intr_rx_parity_err_o (intr_uart0_rx_parity_err),
      .intr_tx_empty_o      (intr_uart0_tx_empty),
      // [0]: fatal_fault
      .alert_tx_o  ( alert_tx[0:0] ),
      .alert_rx_i  ( alert_rx[0:0] ),

      // Inter-module signals
      .lsio_trigger_o(uart0_lsio_trigger),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .tl_i(uart0_tl_req),
      .tl_o(uart0_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  gpio #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[1:1]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .GpioAsyncOn(GpioGpioAsyncOn),
    .GpioAsHwStrapsEn(GpioGpioAsHwStrapsEn)
  ) u_gpio (

      // Input
      .cio_gpio_i    (cio_gpio_gpio_p2d),

      // Output
      .cio_gpio_o    (cio_gpio_gpio_d2p),
      .cio_gpio_en_o (cio_gpio_gpio_en_d2p),

      // Interrupt
      .intr_gpio_o (intr_gpio_gpio),
      // [1]: fatal_fault
      .alert_tx_o  ( alert_tx[1:1] ),
      .alert_rx_i  ( alert_rx[1:1] ),

      // Inter-module signals
      .strap_en_i(pwrmgr_aon_strap),
      .sampled_straps_o(),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .tl_i(gpio_tl_req),
      .tl_o(gpio_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  spi_device #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[2:2]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .SramType(SpiDeviceSramType)
  ) u_spi_device (

      // Input
      .cio_sck_i        (cio_spi_device_sck_p2d),
      .cio_csb_i        (cio_spi_device_csb_p2d),
      .cio_tpm_csb_i    (cio_spi_device_tpm_csb_p2d),
      .cio_sd_i         (cio_spi_device_sd_p2d),

      // Output
      .cio_sd_o         (cio_spi_device_sd_d2p),
      .cio_sd_en_o      (cio_spi_device_sd_en_d2p),

      // Interrupt
      .intr_upload_cmdfifo_not_empty_o (intr_spi_device_upload_cmdfifo_not_empty),
      .intr_upload_payload_not_empty_o (intr_spi_device_upload_payload_not_empty),
      .intr_upload_payload_overflow_o  (intr_spi_device_upload_payload_overflow),
      .intr_readbuf_watermark_o        (intr_spi_device_readbuf_watermark),
      .intr_readbuf_flip_o             (intr_spi_device_readbuf_flip),
      .intr_tpm_header_not_empty_o     (intr_spi_device_tpm_header_not_empty),
      .intr_tpm_rdfifo_cmd_end_o       (intr_spi_device_tpm_rdfifo_cmd_end),
      .intr_tpm_rdfifo_drop_o          (intr_spi_device_tpm_rdfifo_drop),
      // [2]: fatal_fault
      .alert_tx_o  ( alert_tx[2:2] ),
      .alert_rx_i  ( alert_rx[2:2] ),

      // Inter-module signals
      .ram_cfg_sys2spi_i(spi_device_ram_2p_cfg_sys2spi_i),
      .ram_cfg_rsp_sys2spi_o(spi_device_ram_2p_cfg_rsp_sys2spi_o),
      .ram_cfg_spi2sys_i(spi_device_ram_2p_cfg_spi2sys_i),
      .ram_cfg_rsp_spi2sys_o(spi_device_ram_2p_cfg_rsp_spi2sys_o),
      .passthrough_o(spi_device_passthrough_req),
      .passthrough_i(spi_device_passthrough_rsp),
      .mbist_en_i('0),
      .sck_monitor_o(sck_monitor_o),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .tl_i(spi_device_tl_req),
      .tl_o(spi_device_tl_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .scan_clk_i (clkmgr_aon_clocks.clk_io_div2_peri),
      .rst_ni (rstmgr_aon_resets.rst_spi_device_n[rstmgr_pkg::Domain0Sel])
  );
  i2c #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[3:3]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .InputDelayCycles(I2c0InputDelayCycles)
  ) u_i2c0 (

      // Input
      .cio_sda_i    (cio_i2c0_sda_p2d),
      .cio_scl_i    (cio_i2c0_scl_p2d),

      // Output
      .cio_sda_o    (cio_i2c0_sda_d2p),
      .cio_sda_en_o (cio_i2c0_sda_en_d2p),
      .cio_scl_o    (cio_i2c0_scl_d2p),
      .cio_scl_en_o (cio_i2c0_scl_en_d2p),

      // Interrupt
      .intr_fmt_threshold_o    (intr_i2c0_fmt_threshold),
      .intr_rx_threshold_o     (intr_i2c0_rx_threshold),
      .intr_acq_threshold_o    (intr_i2c0_acq_threshold),
      .intr_rx_overflow_o      (intr_i2c0_rx_overflow),
      .intr_controller_halt_o  (intr_i2c0_controller_halt),
      .intr_scl_interference_o (intr_i2c0_scl_interference),
      .intr_sda_interference_o (intr_i2c0_sda_interference),
      .intr_stretch_timeout_o  (intr_i2c0_stretch_timeout),
      .intr_sda_unstable_o     (intr_i2c0_sda_unstable),
      .intr_cmd_complete_o     (intr_i2c0_cmd_complete),
      .intr_tx_stretch_o       (intr_i2c0_tx_stretch),
      .intr_tx_threshold_o     (intr_i2c0_tx_threshold),
      .intr_acq_stretch_o      (intr_i2c0_acq_stretch),
      .intr_unexp_stop_o       (intr_i2c0_unexp_stop),
      .intr_host_timeout_o     (intr_i2c0_host_timeout),
      // [3]: fatal_fault
      .alert_tx_o  ( alert_tx[3:3] ),
      .alert_rx_i  ( alert_rx[3:3] ),

      // Inter-module signals
      .ram_cfg_i(i2c_ram_1p_cfg_i),
      .ram_cfg_rsp_o(i2c_ram_1p_cfg_rsp_o),
      .lsio_trigger_o(i2c0_lsio_trigger),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .tl_i(i2c0_tl_req),
      .tl_o(i2c0_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_i2c0_n[rstmgr_pkg::Domain0Sel])
  );
  rv_timer #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[4:4]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_rv_timer (

      // Interrupt
      .intr_timer_expired_hart0_timer0_o (intr_rv_timer_timer_expired_hart0_timer0),
      // [4]: fatal_fault
      .alert_tx_o  ( alert_tx[4:4] ),
      .alert_rx_i  ( alert_rx[4:4] ),

      // Inter-module signals
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .tl_i(rv_timer_tl_req),
      .tl_o(rv_timer_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_timers),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  otp_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[9:5]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .RndCnstLfsrSeed(RndCnstOtpCtrlLfsrSeed),
    .RndCnstLfsrPerm(RndCnstOtpCtrlLfsrPerm),
    .RndCnstScrmblKeyInit(RndCnstOtpCtrlScrmblKeyInit)
  ) u_otp_ctrl (

      // Interrupt
      .intr_otp_operation_done_o (intr_otp_ctrl_otp_operation_done),
      .intr_otp_error_o          (intr_otp_ctrl_otp_error),
      // [5]: fatal_macro_error
      // [6]: fatal_check_error
      // [7]: fatal_bus_integ_error
      // [8]: fatal_prim_otp_alert
      // [9]: recov_prim_otp_alert
      .alert_tx_o  ( alert_tx[9:5] ),
      .alert_rx_i  ( alert_rx[9:5] ),

      // Inter-module signals
      .edn_o(edn0_edn_req[1]),
      .edn_i(edn0_edn_rsp[1]),
      .pwr_otp_i(pwrmgr_aon_pwr_otp_req),
      .pwr_otp_o(pwrmgr_aon_pwr_otp_rsp),
      .lc_otp_program_i(lc_ctrl_lc_otp_program_req),
      .lc_otp_program_o(lc_ctrl_lc_otp_program_rsp),
      .otp_lc_data_o(otp_ctrl_otp_lc_data),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_creator_seed_sw_rw_en_i(lc_ctrl_lc_creator_seed_sw_rw_en),
      .lc_owner_seed_sw_rw_en_i(lc_ctrl_lc_owner_seed_sw_rw_en),
      .lc_seed_hw_rd_en_i(lc_ctrl_lc_seed_hw_rd_en),
      .lc_check_byp_en_i(lc_ctrl_lc_check_byp_en),
      .otp_keymgr_key_o(otp_ctrl_otp_keymgr_key),
      .sram_otp_key_i(otp_ctrl_sram_otp_key_req),
      .sram_otp_key_o(otp_ctrl_sram_otp_key_rsp),
      .otbn_otp_key_i(otp_ctrl_otbn_otp_key_req),
      .otbn_otp_key_o(otp_ctrl_otbn_otp_key_rsp),
      .otp_broadcast_o(otp_ctrl_otp_broadcast),
      .otp_macro_o(otp_ctrl_otp_macro_req),
      .otp_macro_i(otp_ctrl_otp_macro_rsp),
      .core_tl_i(otp_ctrl_core_tl_req),
      .core_tl_o(otp_ctrl_core_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  otp_macro #(
    .Width(otp_ctrl_macro_pkg::OtpWidth),
    .Depth(otp_ctrl_macro_pkg::OtpDepth),
    .SizeWidth(otp_ctrl_macro_pkg::OtpSizeWidth),
    .MemInitFile(OtpMacroMemInitFile),
    .VendorTestOffset(otp_ctrl_reg_pkg::VendorTestOffset),
    .VendorTestSize(otp_ctrl_reg_pkg::VendorTestSize)
  ) u_otp_macro (

      // Output
      .cio_test_o    (cio_otp_macro_test_d2p),
      .cio_test_en_o (cio_otp_macro_test_en_d2p),

      // Inter-module signals
      .obs_ctrl_i(ast_obs_ctrl),
      .otp_obs_o(otp_obs_o),
      .pwr_seq_o(otp_macro_pwr_seq_o),
      .pwr_seq_h_i(otp_macro_pwr_seq_h_i),
      .ext_voltage_h_io(otp_ext_voltage_h_io),
      .lc_dft_en_i(lc_ctrl_lc_dft_en),
      .test_i(lc_ctrl_lc_otp_vendor_test_req),
      .test_o(lc_ctrl_lc_otp_vendor_test_rsp),
      .otp_i(otp_ctrl_otp_macro_req),
      .otp_o(otp_ctrl_otp_macro_rsp),
      .cfg_i(otp_cfg_i),
      .cfg_rsp_o(otp_cfg_rsp_o),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .prim_tl_i(otp_macro_prim_tl_req),
      .prim_tl_o(otp_macro_prim_tl_rsp),
      .scanmode_i,
      .scan_rst_ni,
      .scan_en_i,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  lc_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[12:10]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .SecVolatileRawUnlockEn(SecLcCtrlVolatileRawUnlockEn),
    .UseDmiInterface(LcCtrlUseDmiInterface),
    .RndCnstLcKeymgrDivInvalid(RndCnstLcCtrlLcKeymgrDivInvalid),
    .RndCnstLcKeymgrDivTestUnlocked(RndCnstLcCtrlLcKeymgrDivTestUnlocked),
    .RndCnstLcKeymgrDivDev(RndCnstLcCtrlLcKeymgrDivDev),
    .RndCnstLcKeymgrDivProduction(RndCnstLcCtrlLcKeymgrDivProduction),
    .RndCnstLcKeymgrDivRma(RndCnstLcCtrlLcKeymgrDivRma),
    .RndCnstInvalidTokens(RndCnstLcCtrlInvalidTokens),
    .SiliconCreatorId(LcCtrlSiliconCreatorId),
    .ProductId(LcCtrlProductId),
    .RevisionId(LcCtrlRevisionId),
    .IdcodeValue(LcCtrlIdcodeValue),
    .NumRmaAckSigs(LcCtrlNumRmaAckSigs),
    .EscNumSeverities(AlertHandlerEscNumSeverities),
    .EscPingCountWidth(AlertHandlerEscPingCountWidth)
  ) u_lc_ctrl (
      // [10]: fatal_prog_error
      // [11]: fatal_state_error
      // [12]: fatal_bus_integ_error
      .alert_tx_o  ( alert_tx[12:10] ),
      .alert_rx_i  ( alert_rx[12:10] ),

      // Inter-module signals
      .jtag_i(jtag_pkg::JTAG_REQ_DEFAULT),
      .jtag_o(),
      .esc_scrap_state0_tx_i(alert_handler_esc_tx[1]),
      .esc_scrap_state0_rx_o(alert_handler_esc_rx[1]),
      .esc_scrap_state1_tx_i(alert_handler_esc_tx[2]),
      .esc_scrap_state1_rx_o(alert_handler_esc_rx[2]),
      .pwr_lc_i(pwrmgr_aon_pwr_lc_req),
      .pwr_lc_o(pwrmgr_aon_pwr_lc_rsp),
      .lc_otp_vendor_test_o(lc_ctrl_lc_otp_vendor_test_req),
      .lc_otp_vendor_test_i(lc_ctrl_lc_otp_vendor_test_rsp),
      .otp_lc_data_i(otp_ctrl_otp_lc_data),
      .lc_otp_program_o(lc_ctrl_lc_otp_program_req),
      .lc_otp_program_i(lc_ctrl_lc_otp_program_rsp),
      .kmac_data_o(kmac_app_req[1]),
      .kmac_data_i(kmac_app_rsp[1]),
      .lc_raw_test_rma_o(lc_ctrl_lc_raw_test_rma),
      .lc_dft_en_o(lc_ctrl_lc_dft_en),
      .lc_nvm_debug_en_o(),
      .lc_hw_debug_clr_o(lc_ctrl_lc_hw_debug_clr),
      .lc_hw_debug_en_o(lc_ctrl_lc_hw_debug_en),
      .lc_cpu_en_o(lc_ctrl_lc_cpu_en),
      .lc_keymgr_en_o(lc_ctrl_lc_keymgr_en),
      .lc_escalate_en_o(lc_ctrl_lc_escalate_en),
      .lc_clk_byp_req_o(lc_ctrl_lc_clk_byp_req),
      .lc_clk_byp_ack_i(lc_ctrl_lc_clk_byp_ack),
      .lc_flash_rma_req_o(lc_ctrl_lc_flash_rma_req),
      .lc_flash_rma_ack_i(otbn_lc_rma_ack),
      .lc_flash_rma_seed_o(),
      .lc_check_byp_en_o(lc_ctrl_lc_check_byp_en),
      .lc_creator_seed_sw_rw_en_o(lc_ctrl_lc_creator_seed_sw_rw_en),
      .lc_owner_seed_sw_rw_en_o(lc_ctrl_lc_owner_seed_sw_rw_en),
      .lc_iso_part_sw_rd_en_o(),
      .lc_iso_part_sw_wr_en_o(),
      .lc_seed_hw_rd_en_o(lc_ctrl_lc_seed_hw_rd_en),
      .lc_keymgr_div_o(lc_ctrl_lc_keymgr_div),
      .otp_device_id_i(lc_ctrl_otp_device_id),
      .otp_manuf_state_i(lc_ctrl_otp_manuf_state),
      .hw_rev_o(),
      .strap_en_override_o(lc_ctrl_strap_en_override),
      .regs_tl_i(lc_ctrl_regs_tl_req),
      .regs_tl_o(lc_ctrl_regs_tl_rsp),
      .dmi_tl_i(lc_ctrl_dmi_tl_req),
      .dmi_tl_o(lc_ctrl_dmi_tl_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_kmac_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
      .rst_kmac_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  alert_handler #(
    .RndCnstLfsrSeed(RndCnstAlertHandlerLfsrSeed),
    .RndCnstLfsrPerm(RndCnstAlertHandlerLfsrPerm),
    .EscNumSeverities(AlertHandlerEscNumSeverities),
    .EscPingCountWidth(AlertHandlerEscPingCountWidth)
  ) u_alert_handler (

      // Interrupt
      .intr_classa_o (intr_alert_handler_classa),
      .intr_classb_o (intr_alert_handler_classb),
      .intr_classc_o (intr_alert_handler_classc),
      .intr_classd_o (intr_alert_handler_classd),

      // Inter-module signals
      .crashdump_o(alert_handler_crashdump),
      .edn_o(edn0_edn_req[3]),
      .edn_i(edn0_edn_rsp[3]),
      .esc_rx_i(alert_handler_esc_rx),
      .esc_tx_o(alert_handler_esc_tx),
      .tl_i(alert_handler_tl_req),
      .tl_o(alert_handler_tl_rsp),
      // alert signals
      .alert_rx_o  ( alert_rx ),
      .alert_tx_i  ( alert_tx ),
      // synchronized clock gated / reset asserted
      // indications for each alert
      .lpg_cg_en_i  ( lpg_cg_en  ),
      .lpg_rst_en_i ( lpg_rst_en ),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_io_div4_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  spi_host #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[13:13]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .NumCS(SpiHost0NumCS)
  ) u_spi_host0 (

      // Input
      .cio_sd_i     (cio_spi_host0_sd_p2d),

      // Output
      .cio_sck_o    (cio_spi_host0_sck_d2p),
      .cio_sck_en_o (cio_spi_host0_sck_en_d2p),
      .cio_csb_o    (cio_spi_host0_csb_d2p),
      .cio_csb_en_o (cio_spi_host0_csb_en_d2p),
      .cio_sd_o     (cio_spi_host0_sd_d2p),
      .cio_sd_en_o  (cio_spi_host0_sd_en_d2p),

      // Interrupt
      .intr_error_o     (intr_spi_host0_error),
      .intr_spi_event_o (intr_spi_host0_spi_event),
      // [13]: fatal_fault
      .alert_tx_o  ( alert_tx[13:13] ),
      .alert_rx_i  ( alert_rx[13:13] ),

      // Inter-module signals
      .passthrough_i(spi_device_passthrough_req),
      .passthrough_o(spi_device_passthrough_rsp),
      .lsio_trigger_o(spi_host0_lsio_trigger),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .tl_i(spi_host0_tl_req),
      .tl_o(spi_host0_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_spi_host0_n[rstmgr_pkg::Domain0Sel])
  );
  pwrmgr #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[14:14]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .EscNumSeverities(AlertHandlerEscNumSeverities),
    .EscPingCountWidth(AlertHandlerEscPingCountWidth)
  ) u_pwrmgr_aon (

      // Interrupt
      .intr_wakeup_o (intr_pwrmgr_aon_wakeup),
      // [14]: fatal_fault
      .alert_tx_o  ( alert_tx[14:14] ),
      .alert_rx_i  ( alert_rx[14:14] ),

      // Inter-module signals
      .boot_status_o(pwrmgr_aon_boot_status),
      .pwr_ast_o(pwrmgr_ast_req_o),
      .pwr_ast_i(pwrmgr_ast_rsp_i),
      .pwr_rst_o(pwrmgr_aon_pwr_rst_req),
      .pwr_rst_i(pwrmgr_aon_pwr_rst_rsp),
      .pwr_clk_o(pwrmgr_aon_pwr_clk_req),
      .pwr_clk_i(pwrmgr_aon_pwr_clk_rsp),
      .pwr_otp_o(pwrmgr_aon_pwr_otp_req),
      .pwr_otp_i(pwrmgr_aon_pwr_otp_rsp),
      .pwr_lc_o(pwrmgr_aon_pwr_lc_req),
      .pwr_lc_i(pwrmgr_aon_pwr_lc_rsp),
      .pwr_flash_i(pwrmgr_pkg::PWR_FLASH_DEFAULT),
      .esc_rst_tx_i(alert_handler_esc_tx[3]),
      .esc_rst_rx_o(alert_handler_esc_rx[3]),
      .pwr_cpu_i(rv_core_ibex_pwrmgr),
      .wakeups_i(pwrmgr_aon_wakeups),
      .rstreqs_i(pwrmgr_aon_rstreqs),
      .ndmreset_req_i(rv_dm_ndmreset_req),
      .strap_o(pwrmgr_aon_strap),
      .low_power_o(pwrmgr_aon_low_power),
      .rom_ctrl_i(pwrmgr_aon_rom_ctrl),
      .fetch_en_o(pwrmgr_aon_fetch_en),
      .lc_dft_en_i(lc_ctrl_lc_dft_en),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .sw_rst_req_i(rstmgr_aon_sw_rst_req),
      .tl_i(pwrmgr_aon_tl_req),
      .tl_o(pwrmgr_aon_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_slow_i (clkmgr_aon_clocks.clk_aon_powerup),
      .clk_lc_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_esc_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .rst_ni (rstmgr_aon_resets.rst_por_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_main_ni (rstmgr_aon_resets.rst_por_aon_n[rstmgr_pkg::Domain0Sel]),
      .rst_lc_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_esc_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_slow_ni (rstmgr_aon_resets.rst_por_aon_n[rstmgr_pkg::DomainAonSel])
  );
  rstmgr #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[16:15]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .SecCheck(SecRstmgrAonCheck),
    .SecMaxSyncDelay(SecRstmgrAonMaxSyncDelay)
  ) u_rstmgr_aon (
      // [15]: fatal_fault
      // [16]: fatal_cnsty_fault
      .alert_tx_o  ( alert_tx[16:15] ),
      .alert_rx_i  ( alert_rx[16:15] ),

      // Inter-module signals
      .por_n_i(por_n_i),
      .pwr_i(pwrmgr_aon_pwr_rst_req),
      .pwr_o(pwrmgr_aon_pwr_rst_rsp),
      .resets_o(rstmgr_aon_resets),
      .rst_en_o(rstmgr_aon_rst_en),
      .alert_dump_i(alert_handler_crashdump),
      .cpu_dump_i(rv_core_ibex_crash_dump),
      .sw_rst_req_o(rstmgr_aon_sw_rst_req),
      .tl_i(rstmgr_aon_tl_req),
      .tl_o(rstmgr_aon_tl_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_por_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_powerup),
      .clk_main_i (clkmgr_aon_clocks.clk_main_powerup),
      .clk_io_i (clkmgr_aon_clocks.clk_io_powerup),
      .clk_io_div2_i (clkmgr_aon_clocks.clk_io_div2_powerup),
      .clk_io_div4_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_por_ni (rstmgr_aon_resets.rst_por_io_div4_n[rstmgr_pkg::DomainAonSel])
  );
  clkmgr #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[18:17]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_clkmgr_aon (
      // [17]: recov_fault
      // [18]: fatal_fault
      .alert_tx_o  ( alert_tx[18:17] ),
      .alert_rx_i  ( alert_rx[18:17] ),

      // Inter-module signals
      .clocks_o(clkmgr_aon_clocks),
      .cg_en_o(clkmgr_aon_cg_en),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .io_clk_byp_req_o(io_clk_byp_req_o),
      .io_clk_byp_ack_i(io_clk_byp_ack_i),
      .all_clk_byp_req_o(all_clk_byp_req_o),
      .all_clk_byp_ack_i(all_clk_byp_ack_i),
      .hi_speed_sel_o(hi_speed_sel_o),
      .div_step_down_req_i(div_step_down_req_i),
      .lc_clk_byp_req_i(lc_ctrl_lc_clk_byp_req),
      .lc_clk_byp_ack_o(lc_ctrl_lc_clk_byp_ack),
      .jitter_en_o(clk_main_jitter_en_o),
      .pwr_i(pwrmgr_aon_pwr_clk_req),
      .pwr_o(pwrmgr_aon_pwr_clk_rsp),
      .idle_i(clkmgr_aon_idle),
      .calib_rdy_i(calib_rdy_i),
      .tl_i(clkmgr_aon_tl_req),
      .tl_o(clkmgr_aon_tl_rsp),
      .scanmode_i,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_main_i (clk_main_i),
      .clk_io_i (clk_io_i),
      .clk_aon_i (clk_aon_i),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_io_div4_shadowed_n[rstmgr_pkg::DomainAonSel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel]),
      .rst_io_ni (rstmgr_aon_resets.rst_lc_io_n[rstmgr_pkg::DomainAonSel]),
      .rst_io_div2_ni (rstmgr_aon_resets.rst_lc_io_div2_n[rstmgr_pkg::DomainAonSel]),
      .rst_io_div4_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_main_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_ni (rstmgr_aon_resets.rst_por_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_io_ni (rstmgr_aon_resets.rst_por_io_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_io_div2_ni (rstmgr_aon_resets.rst_por_io_div2_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_io_div4_ni (rstmgr_aon_resets.rst_por_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_main_ni (rstmgr_aon_resets.rst_por_n[rstmgr_pkg::DomainAonSel])
  );
  pinmux #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[19:19]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .TargetCfg(PinmuxAonTargetCfg)
  ) u_pinmux_aon (
      // [19]: fatal_fault
      .alert_tx_o  ( alert_tx[19:19] ),
      .alert_rx_i  ( alert_rx[19:19] ),

      // Inter-module signals
      .sleep_en_i(pwrmgr_aon_low_power),
      .pin_wkup_req_o(pwrmgr_aon_wakeups[0]),
      .tl_i(pinmux_aon_tl_req),
      .tl_o(pinmux_aon_tl_rsp),

      .periph_to_mio_i      (mio_d2p    ),
      .periph_to_mio_oe_i   (mio_en_d2p ),
      .mio_to_periph_o      (mio_p2d    ),

      .mio_attr_o,
      .mio_out_o,
      .mio_oe_o,
      .mio_in_i,

      .periph_to_dio_i      (dio_d2p    ),
      .periph_to_dio_oe_i   (dio_en_d2p ),
      .dio_to_periph_o      (dio_p2d    ),

      .dio_attr_o,
      .dio_out_o,
      .dio_oe_o,
      .dio_in_i,

      .scanmode_i,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_powerup),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel]),
      .rst_sys_ni (rstmgr_aon_resets.rst_sys_io_div4_n[rstmgr_pkg::DomainAonSel])
  );
  aon_timer #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[20:20]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_aon_timer_aon (

      // Interrupt
      .intr_wkup_timer_expired_o (intr_aon_timer_aon_wkup_timer_expired),
      .intr_wdog_timer_bark_o    (intr_aon_timer_aon_wdog_timer_bark),
      // [20]: fatal_fault
      .alert_tx_o  ( alert_tx[20:20] ),
      .alert_rx_i  ( alert_rx[20:20] ),

      // Inter-module signals
      .nmi_wdog_timer_bark_o(aon_timer_aon_nmi_wdog_timer_bark),
      .wkup_req_o(pwrmgr_aon_wakeups[1]),
      .aon_timer_rst_req_o(pwrmgr_aon_rstreqs[0]),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .sleep_mode_i(pwrmgr_aon_low_power),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .tl_i(aon_timer_aon_tl_req),
      .tl_o(aon_timer_aon_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_timers),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_timers),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel])
  );
  soc_proxy #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[49:21]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_soc_proxy (

      // Input
      .cio_soc_gpi_i    (cio_soc_proxy_soc_gpi_p2d),

      // Output
      .cio_soc_gpo_o    (cio_soc_proxy_soc_gpo_d2p),
      .cio_soc_gpo_en_o (cio_soc_proxy_soc_gpo_en_d2p),

      // Interrupt
      .intr_external_o (intr_soc_proxy_external),
      // [21]: fatal_alert_intg
      // [22]: fatal_alert_external_0
      // [23]: fatal_alert_external_1
      // [24]: fatal_alert_external_2
      // [25]: fatal_alert_external_3
      // [26]: fatal_alert_external_4
      // [27]: fatal_alert_external_5
      // [28]: fatal_alert_external_6
      // [29]: fatal_alert_external_7
      // [30]: fatal_alert_external_8
      // [31]: fatal_alert_external_9
      // [32]: fatal_alert_external_10
      // [33]: fatal_alert_external_11
      // [34]: fatal_alert_external_12
      // [35]: fatal_alert_external_13
      // [36]: fatal_alert_external_14
      // [37]: fatal_alert_external_15
      // [38]: fatal_alert_external_16
      // [39]: fatal_alert_external_17
      // [40]: fatal_alert_external_18
      // [41]: fatal_alert_external_19
      // [42]: fatal_alert_external_20
      // [43]: fatal_alert_external_21
      // [44]: fatal_alert_external_22
      // [45]: fatal_alert_external_23
      // [46]: recov_alert_external_0
      // [47]: recov_alert_external_1
      // [48]: recov_alert_external_2
      // [49]: recov_alert_external_3
      .alert_tx_o  ( alert_tx[49:21] ),
      .alert_rx_i  ( alert_rx[49:21] ),

      // Inter-module signals
      .dma_tl_h2d_i(soc_proxy_dma_tl_h2d),
      .dma_tl_d2h_o(soc_proxy_dma_tl_d2h),
      .misc_tl_h2d_i(ctn_misc_tl_h2d_i),
      .misc_tl_d2h_o(ctn_misc_tl_d2h_o),
      .wkup_internal_req_o(pwrmgr_aon_wakeups[2]),
      .wkup_external_req_o(pwrmgr_aon_wakeups[3]),
      .rst_req_external_o(pwrmgr_aon_rstreqs[1]),
      .ctn_tl_h2d_o(soc_proxy_ctn_tl_h2d),
      .ctn_tl_d2h_i(soc_proxy_ctn_tl_d2h),
      .i2c_lsio_trigger_i(i2c0_lsio_trigger),
      .spi_host_lsio_trigger_i(spi_host0_lsio_trigger),
      .uart_lsio_trigger_i(uart0_lsio_trigger),
      .soc_lsio_trigger_i(soc_lsio_trigger_i),
      .dma_lsio_trigger_o(dma_lsio_trigger),
      .soc_fatal_alert_i(soc_fatal_alert_req_i),
      .soc_fatal_alert_o(soc_fatal_alert_rsp_o),
      .soc_recov_alert_i(soc_recov_alert_req_i),
      .soc_recov_alert_o(soc_recov_alert_rsp_o),
      .soc_wkup_async_i(soc_wkup_async_i),
      .soc_rst_req_async_i(soc_rst_req_async_i),
      .soc_intr_async_i(soc_intr_async_i),
      .soc_gpi_async_o(soc_gpi_async_o),
      .soc_gpo_async_i(soc_gpo_async_i),
      .integrator_id_i(integrator_id_i),
      .core_tl_i(soc_proxy_core_tl_req),
      .core_tl_o(soc_proxy_core_tl_rsp),
      .ctn_tl_i(soc_proxy_ctn_tl_req),
      .ctn_tl_o(soc_proxy_ctn_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_por_ni (rstmgr_aon_resets.rst_por_io_div4_n[rstmgr_pkg::DomainAonSel])
  );
  sram_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[50:50]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .RndCnstSramKey(RndCnstSramCtrlRetAonSramKey),
    .RndCnstSramNonce(RndCnstSramCtrlRetAonSramNonce),
    .RndCnstLfsrSeed(RndCnstSramCtrlRetAonLfsrSeed),
    .RndCnstLfsrPerm(RndCnstSramCtrlRetAonLfsrPerm),
    .MemSizeRam(4096),
    .InstSize(SramCtrlRetAonInstSize),
    .NumRamInst(SramCtrlRetAonNumRamInst),
    .InstrExec(SramCtrlRetAonInstrExec),
    .NumPrinceRoundsHalf(SramCtrlRetAonNumPrinceRoundsHalf),
    .Outstanding(SramCtrlRetAonOutstanding),
    .EccCorrection(SramCtrlRetAonEccCorrection)
  ) u_sram_ctrl_ret_aon (
      // [50]: fatal_error
      .alert_tx_o  ( alert_tx[50:50] ),
      .alert_rx_i  ( alert_rx[50:50] ),
      .racl_policy_sel_ranges_ram_i('{top_racl_pkg::RACL_RANGE_T_DEFAULT}),

      // Inter-module signals
      .sram_otp_key_o(otp_ctrl_sram_otp_key_req[1]),
      .sram_otp_key_i(otp_ctrl_sram_otp_key_rsp[1]),
      .cfg_i(sram_ctrl_ret_aon_ram_1p_cfg_i),
      .cfg_rsp_o(sram_ctrl_ret_aon_ram_1p_cfg_rsp_o),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_hw_debug_en_i(lc_ctrl_pkg::Off),
      .otp_en_sram_ifetch_i(prim_mubi_pkg::MuBi8False),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .sram_rerror_o(),
      .regs_tl_i(sram_ctrl_ret_aon_regs_tl_req),
      .regs_tl_o(sram_ctrl_ret_aon_regs_tl_rsp),
      .ram_tl_i(sram_ctrl_ret_aon_ram_tl_req),
      .ram_tl_o(sram_ctrl_ret_aon_ram_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_infra),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel])
  );
  rv_dm #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[51:51]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .IdcodeValue(RvDmIdcodeValue),
    .UseDmiInterface(RvDmUseDmiInterface),
    .SecVolatileRawUnlockEn(SecRvDmVolatileRawUnlockEn),
    .TlulHostUserRsvdBits(RvDmTlulHostUserRsvdBits)
  ) u_rv_dm (
      // [51]: fatal_fault
      .alert_tx_o  ( alert_tx[51:51] ),
      .alert_rx_i  ( alert_rx[51:51] ),

      // Inter-module signals
      .next_dm_addr_i(rv_dm_next_dm_addr_i),
      .jtag_i(jtag_pkg::JTAG_REQ_DEFAULT),
      .jtag_o(),
      .lc_hw_debug_clr_i(lc_ctrl_lc_hw_debug_clr),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .lc_dft_en_i(lc_ctrl_lc_dft_en),
      .pinmux_hw_debug_en_i(lc_ctrl_pkg::Off),
      .otp_dis_rv_dm_late_debug_i(rv_dm_otp_dis_rv_dm_late_debug),
      .unavailable_i(1'b0),
      .ndmreset_req_o(rv_dm_ndmreset_req),
      .dmactive_o(),
      .debug_req_o(rv_dm_debug_req),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_check_byp_en_i(lc_ctrl_lc_check_byp_en),
      .strap_en_i(pwrmgr_aon_strap),
      .strap_en_override_i(lc_ctrl_strap_en_override),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .sba_tl_h_o(main_tl_rv_dm__sba_req),
      .sba_tl_h_i(main_tl_rv_dm__sba_rsp),
      .regs_tl_d_i(rv_dm_regs_tl_d_req),
      .regs_tl_d_o(rv_dm_regs_tl_d_rsp),
      .mem_tl_d_i(rv_dm_mem_tl_d_req),
      .mem_tl_d_o(rv_dm_mem_tl_d_rsp),
      .dbg_tl_d_i(rv_dm_dbg_tl_d_req),
      .dbg_tl_d_o(rv_dm_dbg_tl_d_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_lc_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_sys_n[rstmgr_pkg::Domain0Sel]),
      .rst_lc_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  rv_plic #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[52:52]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_rv_plic (
      // [52]: fatal_fault
      .alert_tx_o  ( alert_tx[52:52] ),
      .alert_rx_i  ( alert_rx[52:52] ),

      // Inter-module signals
      .irq_o(rv_plic_irq),
      .irq_id_o(),
      .msip_o(rv_plic_msip),
      .tl_i(rv_plic_tl_req),
      .tl_o(rv_plic_tl_rsp),
      .intr_src_i (intr_vector),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  aes #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[54:53]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .AES192Enable(1'b1),
    .SecMasking(SecAesMasking),
    .SecSBoxImpl(SecAesSBoxImpl),
    .SecStartTriggerDelay(SecAesStartTriggerDelay),
    .SecAllowForcingMasks(SecAesAllowForcingMasks),
    .SecSkipPRNGReseeding(SecAesSkipPRNGReseeding),
    .RndCnstClearingLfsrSeed(RndCnstAesClearingLfsrSeed),
    .RndCnstClearingLfsrPerm(RndCnstAesClearingLfsrPerm),
    .RndCnstClearingSharePerm(RndCnstAesClearingSharePerm),
    .RndCnstMaskingLfsrSeed(RndCnstAesMaskingLfsrSeed),
    .RndCnstMaskingLfsrPerm(RndCnstAesMaskingLfsrPerm)
  ) u_aes (
      // [53]: recov_ctrl_update_err
      // [54]: fatal_fault
      .alert_tx_o  ( alert_tx[54:53] ),
      .alert_rx_i  ( alert_rx[54:53] ),

      // Inter-module signals
      .idle_o(clkmgr_aon_idle[0]),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .edn_o(edn0_edn_req[4]),
      .edn_i(edn0_edn_rsp[4]),
      .keymgr_key_i(keymgr_dpe_aes_key),
      .tl_i(aes_tl_req),
      .tl_o(aes_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_aes),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_aes),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  hmac #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[55:55]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_hmac (

      // Interrupt
      .intr_hmac_done_o  (intr_hmac_hmac_done),
      .intr_fifo_empty_o (intr_hmac_fifo_empty),
      .intr_hmac_err_o   (intr_hmac_hmac_err),
      // [55]: fatal_fault
      .alert_tx_o  ( alert_tx[55:55] ),
      .alert_rx_i  ( alert_rx[55:55] ),

      // Inter-module signals
      .idle_o(clkmgr_aon_idle[1]),
      .tl_i(hmac_tl_req),
      .tl_o(hmac_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_hmac),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  kmac #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[57:56]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .EnMasking(KmacEnMasking),
    .SwKeyMasked(KmacSwKeyMasked),
    .SecCmdDelay(SecKmacCmdDelay),
    .SecIdleAcceptSwMsg(SecKmacIdleAcceptSwMsg),
    .NumAppIntf(KmacNumAppIntf),
    .AppCfg(KmacAppCfg),
    .RndCnstLfsrSeed(RndCnstKmacLfsrSeed),
    .RndCnstLfsrPerm(RndCnstKmacLfsrPerm),
    .RndCnstBufferLfsrSeed(RndCnstKmacBufferLfsrSeed),
    .RndCnstMsgPerm(RndCnstKmacMsgPerm)
  ) u_kmac (

      // Interrupt
      .intr_kmac_done_o  (intr_kmac_kmac_done),
      .intr_fifo_empty_o (intr_kmac_fifo_empty),
      .intr_kmac_err_o   (intr_kmac_kmac_err),
      // [56]: recov_operation_err
      // [57]: fatal_fault_err
      .alert_tx_o  ( alert_tx[57:56] ),
      .alert_rx_i  ( alert_rx[57:56] ),

      // Inter-module signals
      .keymgr_key_i(keymgr_dpe_kmac_key),
      .app_i(kmac_app_req),
      .app_o(kmac_app_rsp),
      .entropy_o(edn0_edn_req[2]),
      .entropy_i(edn0_edn_rsp[2]),
      .idle_o(clkmgr_aon_idle[2]),
      .en_masking_o(kmac_en_masking),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .tl_i(kmac_tl_req),
      .tl_o(kmac_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_kmac),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_kmac),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  otbn #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[59:58]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .Stub(OtbnStub),
    .RegFile(OtbnRegFile),
    .RndCnstUrndPrngSeed(RndCnstOtbnUrndPrngSeed),
    .SecMuteUrnd(SecOtbnMuteUrnd),
    .SecSkipUrndReseedAtStart(SecOtbnSkipUrndReseedAtStart),
    .RndCnstOtbnKey(RndCnstOtbnOtbnKey),
    .RndCnstOtbnNonce(RndCnstOtbnOtbnNonce)
  ) u_otbn (

      // Interrupt
      .intr_done_o (intr_otbn_done),
      // [58]: fatal
      // [59]: recov
      .alert_tx_o  ( alert_tx[59:58] ),
      .alert_rx_i  ( alert_rx[59:58] ),

      // Inter-module signals
      .otbn_otp_key_o(otp_ctrl_otbn_otp_key_req),
      .otbn_otp_key_i(otp_ctrl_otbn_otp_key_rsp),
      .edn_rnd_o(edn1_edn_req[0]),
      .edn_rnd_i(edn1_edn_rsp[0]),
      .edn_urnd_o(edn0_edn_req[5]),
      .edn_urnd_i(edn0_edn_rsp[5]),
      .idle_o(clkmgr_aon_idle[3]),
      .ram_cfg_imem_i(otbn_imem_ram_1p_cfg_i),
      .ram_cfg_dmem_i(otbn_dmem_ram_1p_cfg_i),
      .ram_cfg_rsp_imem_o(otbn_imem_ram_1p_cfg_rsp_o),
      .ram_cfg_rsp_dmem_o(otbn_dmem_ram_1p_cfg_rsp_o),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_rma_req_i(lc_ctrl_lc_flash_rma_req),
      .lc_rma_ack_o(otbn_lc_rma_ack),
      .keymgr_key_i(keymgr_dpe_otbn_key),
      .tl_i(otbn_tl_req),
      .tl_o(otbn_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_otbn),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_secure),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  keymgr_dpe #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[61:60]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .KmacEnMasking(KeymgrDpeKmacEnMasking),
    .RndCnstLfsrSeed(RndCnstKeymgrDpeLfsrSeed),
    .RndCnstLfsrPerm(RndCnstKeymgrDpeLfsrPerm),
    .RndCnstRandPerm(RndCnstKeymgrDpeRandPerm),
    .RndCnstRevisionSeed(RndCnstKeymgrDpeRevisionSeed),
    .RndCnstSoftOutputSeed(RndCnstKeymgrDpeSoftOutputSeed),
    .RndCnstHardOutputSeed(RndCnstKeymgrDpeHardOutputSeed),
    .RndCnstAesSeed(RndCnstKeymgrDpeAesSeed),
    .RndCnstKmacSeed(RndCnstKeymgrDpeKmacSeed),
    .RndCnstOtbnSeed(RndCnstKeymgrDpeOtbnSeed),
    .RndCnstNoneSeed(RndCnstKeymgrDpeNoneSeed)
  ) u_keymgr_dpe (

      // Interrupt
      .intr_op_done_o (intr_keymgr_dpe_op_done),
      // [60]: recov_operation_err
      // [61]: fatal_fault_err
      .alert_tx_o  ( alert_tx[61:60] ),
      .alert_rx_i  ( alert_rx[61:60] ),

      // Inter-module signals
      .edn_o(edn0_edn_req[0]),
      .edn_i(edn0_edn_rsp[0]),
      .aes_key_o(keymgr_dpe_aes_key),
      .kmac_key_o(keymgr_dpe_kmac_key),
      .otbn_key_o(keymgr_dpe_otbn_key),
      .kmac_data_o(kmac_app_req[0]),
      .kmac_data_i(kmac_app_rsp[0]),
      .otp_key_i(otp_ctrl_otp_keymgr_key),
      .otp_device_id_i(keymgr_dpe_otp_device_id),
      .lc_keymgr_en_i(lc_ctrl_lc_keymgr_en),
      .lc_keymgr_div_i(lc_ctrl_lc_keymgr_div),
      .rom_digest_i(keymgr_dpe_rom_digest),
      .kmac_en_masking_i(kmac_en_masking),
      .tl_i(keymgr_dpe_tl_req),
      .tl_o(keymgr_dpe_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  csrng #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[63:62]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .RndCnstCsKeymgrDivNonProduction(RndCnstCsrngCsKeymgrDivNonProduction),
    .RndCnstCsKeymgrDivProduction(RndCnstCsrngCsKeymgrDivProduction),
    .SBoxImpl(CsrngSBoxImpl)
  ) u_csrng (

      // Interrupt
      .intr_cs_cmd_req_done_o (intr_csrng_cs_cmd_req_done),
      .intr_cs_entropy_req_o  (intr_csrng_cs_entropy_req),
      .intr_cs_hw_inst_exc_o  (intr_csrng_cs_hw_inst_exc),
      .intr_cs_fatal_err_o    (intr_csrng_cs_fatal_err),
      // [62]: recov_alert
      // [63]: fatal_alert
      .alert_tx_o  ( alert_tx[63:62] ),
      .alert_rx_i  ( alert_rx[63:62] ),

      // Inter-module signals
      .csrng_cmd_i(csrng_csrng_cmd_req),
      .csrng_cmd_o(csrng_csrng_cmd_rsp),
      .entropy_src_hw_if_o(csrng_entropy_src_hw_if_req),
      .entropy_src_hw_if_i(csrng_entropy_src_hw_if_rsp),
      .cs_aes_halt_i(csrng_cs_aes_halt_req),
      .cs_aes_halt_o(csrng_cs_aes_halt_rsp),
      .otp_en_csrng_sw_app_read_i(csrng_otp_en_csrng_sw_app_read),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .tl_i(csrng_tl_req),
      .tl_o(csrng_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  entropy_src #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[65:64]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .RngBusWidth(EntropySrcRngBusWidth),
    .RngBusBitSelWidth(EntropySrcRngBusBitSelWidth),
    .HealthTestWindowWidth(EntropySrcHealthTestWindowWidth),
    .EsFifoDepth(EntropySrcEsFifoDepth),
    .DistrFifoDepth(EntropySrcDistrFifoDepth),
    .Stub(EntropySrcStub)
  ) u_entropy_src (

      // Interrupt
      .intr_es_entropy_valid_o      (intr_entropy_src_es_entropy_valid),
      .intr_es_health_test_failed_o (intr_entropy_src_es_health_test_failed),
      .intr_es_observe_fifo_ready_o (intr_entropy_src_es_observe_fifo_ready),
      .intr_es_fatal_err_o          (intr_entropy_src_es_fatal_err),
      // [64]: recov_alert
      // [65]: fatal_alert
      .alert_tx_o  ( alert_tx[65:64] ),
      .alert_rx_i  ( alert_rx[65:64] ),

      // Inter-module signals
      .entropy_src_hw_if_i(csrng_entropy_src_hw_if_req),
      .entropy_src_hw_if_o(csrng_entropy_src_hw_if_rsp),
      .cs_aes_halt_o(csrng_cs_aes_halt_req),
      .cs_aes_halt_i(csrng_cs_aes_halt_rsp),
      .entropy_src_rng_enable_o(es_rng_enable_o),
      .entropy_src_rng_valid_i(es_rng_valid_i),
      .entropy_src_rng_bits_i(es_rng_bit_i),
      .entropy_src_xht_valid_o(),
      .entropy_src_xht_bits_o(),
      .entropy_src_xht_bit_sel_o(),
      .entropy_src_xht_health_test_window_o(),
      .entropy_src_xht_meta_o(),
      .entropy_src_xht_meta_i(entropy_src_pkg::ENTROPY_SRC_XHT_META_RSP_DEFAULT),
      .otp_en_entropy_src_fw_read_i(prim_mubi_pkg::MuBi8True),
      .otp_en_entropy_src_fw_over_i(prim_mubi_pkg::MuBi8True),
      .rng_fips_o(es_rng_fips_o),
      .tl_i(entropy_src_tl_req),
      .tl_o(entropy_src_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  edn #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[67:66]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_edn0 (

      // Interrupt
      .intr_edn_cmd_req_done_o (intr_edn0_edn_cmd_req_done),
      .intr_edn_fatal_err_o    (intr_edn0_edn_fatal_err),
      // [66]: recov_alert
      // [67]: fatal_alert
      .alert_tx_o  ( alert_tx[67:66] ),
      .alert_rx_i  ( alert_rx[67:66] ),

      // Inter-module signals
      .csrng_cmd_o(csrng_csrng_cmd_req[0]),
      .csrng_cmd_i(csrng_csrng_cmd_rsp[0]),
      .edn_i(edn0_edn_req),
      .edn_o(edn0_edn_rsp),
      .tl_i(edn0_tl_req),
      .tl_o(edn0_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  edn #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[69:68]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_edn1 (

      // Interrupt
      .intr_edn_cmd_req_done_o (intr_edn1_edn_cmd_req_done),
      .intr_edn_fatal_err_o    (intr_edn1_edn_fatal_err),
      // [68]: recov_alert
      // [69]: fatal_alert
      .alert_tx_o  ( alert_tx[69:68] ),
      .alert_rx_i  ( alert_rx[69:68] ),

      // Inter-module signals
      .csrng_cmd_o(csrng_csrng_cmd_req[1]),
      .csrng_cmd_i(csrng_csrng_cmd_rsp[1]),
      .edn_i(edn1_edn_req),
      .edn_o(edn1_edn_rsp),
      .tl_i(edn1_tl_req),
      .tl_o(edn1_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  sram_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[70:70]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .RndCnstSramKey(RndCnstSramCtrlMainSramKey),
    .RndCnstSramNonce(RndCnstSramCtrlMainSramNonce),
    .RndCnstLfsrSeed(RndCnstSramCtrlMainLfsrSeed),
    .RndCnstLfsrPerm(RndCnstSramCtrlMainLfsrPerm),
    .MemSizeRam(65536),
    .InstSize(SramCtrlMainInstSize),
    .NumRamInst(SramCtrlMainNumRamInst),
    .InstrExec(SramCtrlMainInstrExec),
    .NumPrinceRoundsHalf(SramCtrlMainNumPrinceRoundsHalf),
    .Outstanding(SramCtrlMainOutstanding),
    .EccCorrection(SramCtrlMainEccCorrection)
  ) u_sram_ctrl_main (
      // [70]: fatal_error
      .alert_tx_o  ( alert_tx[70:70] ),
      .alert_rx_i  ( alert_rx[70:70] ),
      .racl_policy_sel_ranges_ram_i('{top_racl_pkg::RACL_RANGE_T_DEFAULT}),

      // Inter-module signals
      .sram_otp_key_o(otp_ctrl_sram_otp_key_req[0]),
      .sram_otp_key_i(otp_ctrl_sram_otp_key_rsp[0]),
      .cfg_i(sram_ctrl_main_ram_1p_cfg_i),
      .cfg_rsp_o(sram_ctrl_main_ram_1p_cfg_rsp_o),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .otp_en_sram_ifetch_i(sram_ctrl_main_otp_en_sram_ifetch),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .sram_rerror_o(),
      .regs_tl_i(sram_ctrl_main_regs_tl_req),
      .regs_tl_o(sram_ctrl_main_regs_tl_rsp),
      .ram_tl_i(sram_ctrl_main_ram_tl_req),
      .ram_tl_o(sram_ctrl_main_ram_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  sram_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[71:71]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .RndCnstSramKey(RndCnstSramCtrlMboxSramKey),
    .RndCnstSramNonce(RndCnstSramCtrlMboxSramNonce),
    .RndCnstLfsrSeed(RndCnstSramCtrlMboxLfsrSeed),
    .RndCnstLfsrPerm(RndCnstSramCtrlMboxLfsrPerm),
    .MemSizeRam(4096),
    .InstSize(SramCtrlMboxInstSize),
    .NumRamInst(SramCtrlMboxNumRamInst),
    .InstrExec(SramCtrlMboxInstrExec),
    .NumPrinceRoundsHalf(SramCtrlMboxNumPrinceRoundsHalf),
    .Outstanding(SramCtrlMboxOutstanding),
    .EccCorrection(SramCtrlMboxEccCorrection)
  ) u_sram_ctrl_mbox (
      // [71]: fatal_error
      .alert_tx_o  ( alert_tx[71:71] ),
      .alert_rx_i  ( alert_rx[71:71] ),
      .racl_policy_sel_ranges_ram_i('{top_racl_pkg::RACL_RANGE_T_DEFAULT}),

      // Inter-module signals
      .sram_otp_key_o(otp_ctrl_sram_otp_key_req[2]),
      .sram_otp_key_i(otp_ctrl_sram_otp_key_rsp[2]),
      .cfg_i(sram_ctrl_mbox_ram_1p_cfg_i),
      .cfg_rsp_o(sram_ctrl_mbox_ram_1p_cfg_rsp_o),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_hw_debug_en_i(lc_ctrl_pkg::Off),
      .otp_en_sram_ifetch_i(prim_mubi_pkg::MuBi8False),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .sram_rerror_o(),
      .regs_tl_i(sram_ctrl_mbox_regs_tl_req),
      .regs_tl_o(sram_ctrl_mbox_regs_tl_rsp),
      .ram_tl_i(sram_ctrl_mbox_ram_tl_req),
      .ram_tl_o(sram_ctrl_mbox_ram_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  rom_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[72:72]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .BootRomInitFile(RomCtrl0BootRomInitFile),
    .RndCnstScrNonce(RndCnstRomCtrl0ScrNonce),
    .RndCnstScrKey(RndCnstRomCtrl0ScrKey),
    .SecDisableScrambling(SecRomCtrl0DisableScrambling),
    .MemSizeRom(32768)
  ) u_rom_ctrl0 (
      // [72]: fatal
      .alert_tx_o  ( alert_tx[72:72] ),
      .alert_rx_i  ( alert_rx[72:72] ),

      // Inter-module signals
      .rom_cfg_i(rom_ctrl0_cfg_i),
      .pwrmgr_data_o(pwrmgr_aon_rom_ctrl[0]),
      .keymgr_data_o(keymgr_dpe_rom_digest[0]),
      .kmac_data_o(kmac_app_req[2]),
      .kmac_data_i(kmac_app_rsp[2]),
      .regs_tl_i(rom_ctrl0_regs_tl_req),
      .regs_tl_o(rom_ctrl0_regs_tl_rsp),
      .rom_tl_i(rom_ctrl0_rom_tl_req),
      .rom_tl_o(rom_ctrl0_rom_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  rom_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[73:73]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .BootRomInitFile(RomCtrl1BootRomInitFile),
    .RndCnstScrNonce(RndCnstRomCtrl1ScrNonce),
    .RndCnstScrKey(RndCnstRomCtrl1ScrKey),
    .SecDisableScrambling(SecRomCtrl1DisableScrambling),
    .MemSizeRom(65536)
  ) u_rom_ctrl1 (
      // [73]: fatal
      .alert_tx_o  ( alert_tx[73:73] ),
      .alert_rx_i  ( alert_rx[73:73] ),

      // Inter-module signals
      .rom_cfg_i(rom_ctrl1_cfg_i),
      .pwrmgr_data_o(pwrmgr_aon_rom_ctrl[1]),
      .keymgr_data_o(keymgr_dpe_rom_digest[1]),
      .kmac_data_o(kmac_app_req[3]),
      .kmac_data_i(kmac_app_rsp[3]),
      .regs_tl_i(rom_ctrl1_regs_tl_req),
      .regs_tl_o(rom_ctrl1_regs_tl_rsp),
      .rom_tl_i(rom_ctrl1_rom_tl_req),
      .rom_tl_o(rom_ctrl1_rom_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  dma #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[74:74]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .EnableDataIntgGen(DmaEnableDataIntgGen),
    .EnableRspDataIntgCheck(DmaEnableRspDataIntgCheck),
    .TlUserRsvd(DmaTlUserRsvd),
    .SysRacl(DmaSysRacl),
    .OtAgentId(DmaOtAgentId)
  ) u_dma (

      // Interrupt
      .intr_dma_done_o       (intr_dma_dma_done),
      .intr_dma_chunk_done_o (intr_dma_dma_chunk_done),
      .intr_dma_error_o      (intr_dma_dma_error),
      // [74]: fatal_fault
      .alert_tx_o  ( alert_tx[74:74] ),
      .alert_rx_i  ( alert_rx[74:74] ),

      // Inter-module signals
      .lsio_trigger_i(dma_lsio_trigger),
      .sys_o(dma_sys_req_o),
      .sys_i(dma_sys_rsp_i),
      .ctn_tl_h2d_o(soc_proxy_dma_tl_h2d),
      .ctn_tl_d2h_i(soc_proxy_dma_tl_d2h),
      .racl_policies_i(top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
      .racl_error_o(),
      .host_tl_h_o(main_tl_dma__host_req),
      .host_tl_h_i(main_tl_dma__host_rsp),
      .tl_d_i(dma_tl_d_req),
      .tl_d_o(dma_tl_d_rsp),
      .scanmode_i,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX0_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX0_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX0_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[76:75]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx0 (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx0_mbx_ready),
      .intr_mbx_abort_o (intr_mbx0_mbx_abort),
      .intr_mbx_error_o (intr_mbx0_mbx_error),
      // [75]: fatal_fault
      // [76]: recov_fault
      .alert_tx_o  ( alert_tx[76:75] ),
      .alert_rx_i  ( alert_rx[76:75] ),

      // Inter-module signals
      .doe_intr_support_o(mbx0_doe_intr_support_o),
      .doe_intr_en_o(mbx0_doe_intr_en_o),
      .doe_intr_o(mbx0_doe_intr_o),
      .doe_async_msg_support_o(mbx0_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[0]),
      .sram_tl_h_o(main_tl_mbx0__sram_req),
      .sram_tl_h_i(main_tl_mbx0__sram_rsp),
      .core_tl_d_i(mbx0_core_tl_d_req),
      .core_tl_d_o(mbx0_core_tl_d_rsp),
      .soc_tl_d_i(mbx0_soc_tl_d_req),
      .soc_tl_d_o(mbx0_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX1_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX1_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX1_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[78:77]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx1 (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx1_mbx_ready),
      .intr_mbx_abort_o (intr_mbx1_mbx_abort),
      .intr_mbx_error_o (intr_mbx1_mbx_error),
      // [77]: fatal_fault
      // [78]: recov_fault
      .alert_tx_o  ( alert_tx[78:77] ),
      .alert_rx_i  ( alert_rx[78:77] ),

      // Inter-module signals
      .doe_intr_support_o(mbx1_doe_intr_support_o),
      .doe_intr_en_o(mbx1_doe_intr_en_o),
      .doe_intr_o(mbx1_doe_intr_o),
      .doe_async_msg_support_o(mbx1_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[1]),
      .sram_tl_h_o(main_tl_mbx1__sram_req),
      .sram_tl_h_i(main_tl_mbx1__sram_rsp),
      .core_tl_d_i(mbx1_core_tl_d_req),
      .core_tl_d_o(mbx1_core_tl_d_rsp),
      .soc_tl_d_i(mbx1_soc_tl_d_req),
      .soc_tl_d_o(mbx1_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX2_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX2_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX2_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[80:79]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx2 (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx2_mbx_ready),
      .intr_mbx_abort_o (intr_mbx2_mbx_abort),
      .intr_mbx_error_o (intr_mbx2_mbx_error),
      // [79]: fatal_fault
      // [80]: recov_fault
      .alert_tx_o  ( alert_tx[80:79] ),
      .alert_rx_i  ( alert_rx[80:79] ),

      // Inter-module signals
      .doe_intr_support_o(mbx2_doe_intr_support_o),
      .doe_intr_en_o(mbx2_doe_intr_en_o),
      .doe_intr_o(mbx2_doe_intr_o),
      .doe_async_msg_support_o(mbx2_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[2]),
      .sram_tl_h_o(main_tl_mbx2__sram_req),
      .sram_tl_h_i(main_tl_mbx2__sram_rsp),
      .core_tl_d_i(mbx2_core_tl_d_req),
      .core_tl_d_o(mbx2_core_tl_d_rsp),
      .soc_tl_d_i(mbx2_soc_tl_d_req),
      .soc_tl_d_o(mbx2_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX3_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX3_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX3_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[82:81]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx3 (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx3_mbx_ready),
      .intr_mbx_abort_o (intr_mbx3_mbx_abort),
      .intr_mbx_error_o (intr_mbx3_mbx_error),
      // [81]: fatal_fault
      // [82]: recov_fault
      .alert_tx_o  ( alert_tx[82:81] ),
      .alert_rx_i  ( alert_rx[82:81] ),

      // Inter-module signals
      .doe_intr_support_o(mbx3_doe_intr_support_o),
      .doe_intr_en_o(mbx3_doe_intr_en_o),
      .doe_intr_o(mbx3_doe_intr_o),
      .doe_async_msg_support_o(mbx3_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[3]),
      .sram_tl_h_o(main_tl_mbx3__sram_req),
      .sram_tl_h_i(main_tl_mbx3__sram_rsp),
      .core_tl_d_i(mbx3_core_tl_d_req),
      .core_tl_d_o(mbx3_core_tl_d_rsp),
      .soc_tl_d_i(mbx3_soc_tl_d_req),
      .soc_tl_d_o(mbx3_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX4_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX4_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX4_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[84:83]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx4 (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx4_mbx_ready),
      .intr_mbx_abort_o (intr_mbx4_mbx_abort),
      .intr_mbx_error_o (intr_mbx4_mbx_error),
      // [83]: fatal_fault
      // [84]: recov_fault
      .alert_tx_o  ( alert_tx[84:83] ),
      .alert_rx_i  ( alert_rx[84:83] ),

      // Inter-module signals
      .doe_intr_support_o(mbx4_doe_intr_support_o),
      .doe_intr_en_o(mbx4_doe_intr_en_o),
      .doe_intr_o(mbx4_doe_intr_o),
      .doe_async_msg_support_o(mbx4_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[4]),
      .sram_tl_h_o(main_tl_mbx4__sram_req),
      .sram_tl_h_i(main_tl_mbx4__sram_rsp),
      .core_tl_d_i(mbx4_core_tl_d_req),
      .core_tl_d_o(mbx4_core_tl_d_rsp),
      .soc_tl_d_i(mbx4_soc_tl_d_req),
      .soc_tl_d_o(mbx4_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX5_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX5_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX5_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[86:85]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx5 (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx5_mbx_ready),
      .intr_mbx_abort_o (intr_mbx5_mbx_abort),
      .intr_mbx_error_o (intr_mbx5_mbx_error),
      // [85]: fatal_fault
      // [86]: recov_fault
      .alert_tx_o  ( alert_tx[86:85] ),
      .alert_rx_i  ( alert_rx[86:85] ),

      // Inter-module signals
      .doe_intr_support_o(mbx5_doe_intr_support_o),
      .doe_intr_en_o(mbx5_doe_intr_en_o),
      .doe_intr_o(mbx5_doe_intr_o),
      .doe_async_msg_support_o(mbx5_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[5]),
      .sram_tl_h_o(main_tl_mbx5__sram_req),
      .sram_tl_h_i(main_tl_mbx5__sram_rsp),
      .core_tl_d_i(mbx5_core_tl_d_req),
      .core_tl_d_o(mbx5_core_tl_d_rsp),
      .soc_tl_d_i(mbx5_soc_tl_d_req),
      .soc_tl_d_o(mbx5_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX6_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX6_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX6_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[88:87]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx6 (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx6_mbx_ready),
      .intr_mbx_abort_o (intr_mbx6_mbx_abort),
      .intr_mbx_error_o (intr_mbx6_mbx_error),
      // [87]: fatal_fault
      // [88]: recov_fault
      .alert_tx_o  ( alert_tx[88:87] ),
      .alert_rx_i  ( alert_rx[88:87] ),

      // Inter-module signals
      .doe_intr_support_o(mbx6_doe_intr_support_o),
      .doe_intr_en_o(mbx6_doe_intr_en_o),
      .doe_intr_o(mbx6_doe_intr_o),
      .doe_async_msg_support_o(mbx6_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[6]),
      .sram_tl_h_o(main_tl_mbx6__sram_req),
      .sram_tl_h_i(main_tl_mbx6__sram_rsp),
      .core_tl_d_i(mbx6_core_tl_d_req),
      .core_tl_d_o(mbx6_core_tl_d_rsp),
      .soc_tl_d_i(mbx6_soc_tl_d_req),
      .soc_tl_d_o(mbx6_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX_JTAG_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX_JTAG_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX_JTAG_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[90:89]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx_jtag (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx_jtag_mbx_ready),
      .intr_mbx_abort_o (intr_mbx_jtag_mbx_abort),
      .intr_mbx_error_o (intr_mbx_jtag_mbx_error),
      // [89]: fatal_fault
      // [90]: recov_fault
      .alert_tx_o  ( alert_tx[90:89] ),
      .alert_rx_i  ( alert_rx[90:89] ),

      // Inter-module signals
      .doe_intr_support_o(mbx_jtag_doe_intr_support_o),
      .doe_intr_en_o(mbx_jtag_doe_intr_en_o),
      .doe_intr_o(mbx_jtag_doe_intr_o),
      .doe_async_msg_support_o(mbx_jtag_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[7]),
      .sram_tl_h_o(main_tl_mbx_jtag__sram_req),
      .sram_tl_h_i(main_tl_mbx_jtag__sram_rsp),
      .core_tl_d_i(mbx_jtag_core_tl_d_req),
      .core_tl_d_o(mbx_jtag_core_tl_d_rsp),
      .soc_tl_d_i(mbx_jtag_soc_tl_d_req),
      .soc_tl_d_o(mbx_jtag_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX_PCIE0_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX_PCIE0_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX_PCIE0_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[92:91]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx_pcie0 (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx_pcie0_mbx_ready),
      .intr_mbx_abort_o (intr_mbx_pcie0_mbx_abort),
      .intr_mbx_error_o (intr_mbx_pcie0_mbx_error),
      // [91]: fatal_fault
      // [92]: recov_fault
      .alert_tx_o  ( alert_tx[92:91] ),
      .alert_rx_i  ( alert_rx[92:91] ),

      // Inter-module signals
      .doe_intr_support_o(mbx_pcie0_doe_intr_support_o),
      .doe_intr_en_o(mbx_pcie0_doe_intr_en_o),
      .doe_intr_o(mbx_pcie0_doe_intr_o),
      .doe_async_msg_support_o(mbx_pcie0_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[8]),
      .sram_tl_h_o(main_tl_mbx_pcie0__sram_req),
      .sram_tl_h_i(main_tl_mbx_pcie0__sram_rsp),
      .core_tl_d_i(mbx_pcie0_core_tl_d_req),
      .core_tl_d_o(mbx_pcie0_core_tl_d_rsp),
      .soc_tl_d_i(mbx_pcie0_soc_tl_d_req),
      .soc_tl_d_o(mbx_pcie0_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  mbx #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVecSoc(RACL_POLICY_SEL_VEC_MBX_PCIE1_SOC),
    .RaclPolicySelWinSocWdata(RACL_POLICY_SEL_WIN_MBX_PCIE1_SOC_WDATA),
    .RaclPolicySelWinSocRdata(RACL_POLICY_SEL_WIN_MBX_PCIE1_SOC_RDATA),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[94:93]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_mbx_pcie1 (

      // Interrupt
      .intr_mbx_ready_o (intr_mbx_pcie1_mbx_ready),
      .intr_mbx_abort_o (intr_mbx_pcie1_mbx_abort),
      .intr_mbx_error_o (intr_mbx_pcie1_mbx_error),
      // [93]: fatal_fault
      // [94]: recov_fault
      .alert_tx_o  ( alert_tx[94:93] ),
      .alert_rx_i  ( alert_rx[94:93] ),

      // Inter-module signals
      .doe_intr_support_o(mbx_pcie1_doe_intr_support_o),
      .doe_intr_en_o(mbx_pcie1_doe_intr_en_o),
      .doe_intr_o(mbx_pcie1_doe_intr_o),
      .doe_async_msg_support_o(mbx_pcie1_doe_async_msg_support_o),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[9]),
      .sram_tl_h_o(main_tl_mbx_pcie1__sram_req),
      .sram_tl_h_i(main_tl_mbx_pcie1__sram_rsp),
      .core_tl_d_i(mbx_pcie1_core_tl_d_req),
      .core_tl_d_o(mbx_pcie1_core_tl_d_rsp),
      .soc_tl_d_i(mbx_pcie1_soc_tl_d_req),
      .soc_tl_d_o(mbx_pcie1_soc_tl_d_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  soc_dbg_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[96:95]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles)
  ) u_soc_dbg_ctrl (
      // [95]: fatal_fault
      // [96]: recov_ctrl_update_err
      .alert_tx_o  ( alert_tx[96:95] ),
      .alert_rx_i  ( alert_rx[96:95] ),

      // Inter-module signals
      .boot_status_i(pwrmgr_aon_boot_status),
      .soc_dbg_state_i(soc_dbg_ctrl_soc_dbg_state),
      .soc_dbg_policy_bus_o(soc_dbg_policy_bus_o),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .lc_dft_en_i(lc_ctrl_lc_dft_en),
      .lc_raw_test_rma_i(lc_ctrl_lc_raw_test_rma),
      .halt_cpu_boot_i(debug_halt_cpu_boot_i),
      .continue_cpu_boot_o(pwrmgr_aon_rom_ctrl[2]),
      .core_tl_i(soc_dbg_ctrl_core_tl_req),
      .core_tl_o(soc_dbg_ctrl_core_tl_rsp),
      .jtag_tl_i(soc_dbg_ctrl_jtag_tl_req),
      .jtag_tl_o(soc_dbg_ctrl_jtag_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_io_div4_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  racl_ctrl #(
    .RaclErrorRsp(1'b1),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[98:97]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .NumSubscribingIps(RaclCtrlNumSubscribingIps),
    .NumExternalSubscribingIps(RaclCtrlNumExternalSubscribingIps)
  ) u_racl_ctrl (

      // Interrupt
      .intr_racl_error_o (intr_racl_ctrl_racl_error),
      // [97]: fatal_fault
      // [98]: recov_ctrl_update_err
      .alert_tx_o  ( alert_tx[98:97] ),
      .alert_rx_i  ( alert_rx[98:97] ),

      // Inter-module signals
      .racl_policies_o(racl_ctrl_racl_policies),
      .racl_error_i(racl_ctrl_racl_error),
      .racl_error_external_i(racl_error_i),
      .tl_i(racl_ctrl_tl_req),
      .tl_o(racl_ctrl_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  ac_range_check #(
    .EnableRacl(1'b1),
    .RaclErrorRsp(top_racl_pkg::ErrorRsp),
    .RaclPolicySelVec(RACL_POLICY_SEL_VEC_AC_RANGE_CHECK),
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[100:99]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .RangeCheckErrorRsp(AcRangeCheckRangeCheckErrorRsp)
  ) u_ac_range_check (

      // Interrupt
      .intr_deny_cnt_reached_o (intr_ac_range_check_deny_cnt_reached),
      // [99]: recov_ctrl_update_err
      // [100]: fatal_fault
      .alert_tx_o  ( alert_tx[100:99] ),
      .alert_rx_i  ( alert_rx[100:99] ),

      // Inter-module signals
      .range_check_overwrite_i(ac_range_check_overwrite_i),
      .ctn_tl_h2d_i(soc_proxy_ctn_tl_h2d),
      .ctn_tl_d2h_o(soc_proxy_ctn_tl_d2h),
      .ctn_filtered_tl_h2d_o(ctn_tl_h2d_o),
      .ctn_filtered_tl_d2h_i(ctn_tl_d2h_i),
      .racl_policies_i(racl_ctrl_racl_policies),
      .racl_error_o(racl_ctrl_racl_error[10]),
      .tl_i(ac_range_check_tl_req),
      .tl_o(ac_range_check_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  rv_core_ibex #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[104:101]),
    .AlertSkewCycles(top_pkg::AlertSkewCycles),
    .RndCnstLfsrSeed(RndCnstRvCoreIbexLfsrSeed),
    .RndCnstLfsrPerm(RndCnstRvCoreIbexLfsrPerm),
    .RndCnstIbexKeyDefault(RndCnstRvCoreIbexIbexKeyDefault),
    .RndCnstIbexNonceDefault(RndCnstRvCoreIbexIbexNonceDefault),
    .NEscalationSeverities(AlertHandlerEscNumSeverities),
    .WidthPingCounter(AlertHandlerEscPingCountWidth),
    .PMPEnable(RvCoreIbexPMPEnable),
    .PMPGranularity(RvCoreIbexPMPGranularity),
    .PMPNumRegions(RvCoreIbexPMPNumRegions),
    .MHPMCounterNum(RvCoreIbexMHPMCounterNum),
    .MHPMCounterWidth(RvCoreIbexMHPMCounterWidth),
    .PMPRstCfg(RvCoreIbexPMPRstCfg),
    .PMPRstAddr(RvCoreIbexPMPRstAddr),
    .PMPRstMsecCfg(RvCoreIbexPMPRstMsecCfg),
    .RV32E(RvCoreIbexRV32E),
    .RV32M(RvCoreIbexRV32M),
    .RV32B(RvCoreIbexRV32B),
    .RegFile(RvCoreIbexRegFile),
    .BranchTargetALU(RvCoreIbexBranchTargetALU),
    .WritebackStage(RvCoreIbexWritebackStage),
    .ICache(RvCoreIbexICache),
    .ICacheECC(RvCoreIbexICacheECC),
    .ICacheScramble(RvCoreIbexICacheScramble),
    .ICacheNWays(RvCoreIbexICacheNWays),
    .BranchPredictor(RvCoreIbexBranchPredictor),
    .DbgTriggerEn(RvCoreIbexDbgTriggerEn),
    .DbgHwBreakNum(RvCoreIbexDbgHwBreakNum),
    .SecureIbex(RvCoreIbexSecureIbex),
    .DmBaseAddr(RvCoreIbexDmBaseAddr),
    .DmAddrMask(RvCoreIbexDmAddrMask),
    .DmHaltAddr(RvCoreIbexDmHaltAddr),
    .DmExceptionAddr(RvCoreIbexDmExceptionAddr),
    .PipeLine(RvCoreIbexPipeLine),
    .TlulHostUserRsvdBits(RvCoreIbexTlulHostUserRsvdBits),
    .CsrMvendorId(RvCoreIbexCsrMvendorId),
    .CsrMimpId(RvCoreIbexCsrMimpId)
  ) u_rv_core_ibex (
      // [101]: fatal_sw_err
      // [102]: recov_sw_err
      // [103]: fatal_hw_err
      // [104]: recov_hw_err
      .alert_tx_o  ( alert_tx[104:101] ),
      .alert_rx_i  ( alert_rx[104:101] ),

      // Inter-module signals
      .rst_cpu_n_o(),
      .ram_cfg_icache_tag_i(rv_core_ibex_icache_tag_ram_1p_cfg_i),
      .ram_cfg_rsp_icache_tag_o(rv_core_ibex_icache_tag_ram_1p_cfg_rsp_o),
      .ram_cfg_icache_data_i(rv_core_ibex_icache_data_ram_1p_cfg_i),
      .ram_cfg_rsp_icache_data_o(rv_core_ibex_icache_data_ram_1p_cfg_rsp_o),
      .hart_id_i(rv_core_ibex_hart_id),
      .boot_addr_i(rv_core_ibex_boot_addr),
      .irq_software_i(rv_plic_msip),
      .irq_timer_i(rv_core_ibex_irq_timer),
      .irq_external_i(rv_plic_irq),
      .esc_tx_i(alert_handler_esc_tx[0]),
      .esc_rx_o(alert_handler_esc_rx[0]),
      .debug_req_i(rv_dm_debug_req),
      .crash_dump_o(rv_core_ibex_crash_dump),
      .lc_cpu_en_i(lc_ctrl_lc_cpu_en),
      .pwrmgr_cpu_en_i(pwrmgr_aon_fetch_en),
      .pwrmgr_o(rv_core_ibex_pwrmgr),
      .nmi_wdog_i(aon_timer_aon_nmi_wdog_timer_bark),
      .edn_o(edn0_edn_req[6]),
      .edn_i(edn0_edn_rsp[6]),
      .icache_otp_key_o(otp_ctrl_sram_otp_key_req[3]),
      .icache_otp_key_i(otp_ctrl_sram_otp_key_rsp[3]),
      .fpga_info_i(fpga_info_i),
      .corei_tl_h_o(main_tl_rv_core_ibex__corei_req),
      .corei_tl_h_i(main_tl_rv_core_ibex__corei_rsp),
      .cored_tl_h_o(main_tl_rv_core_ibex__cored_req),
      .cored_tl_h_i(main_tl_rv_core_ibex__cored_rsp),
      .cfg_tl_d_i(rv_core_ibex_cfg_tl_d_req),
      .cfg_tl_d_o(rv_core_ibex_cfg_tl_d_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_esc_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_esc_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );


  // interrupt assignments
  assign intr_vector = {
      intr_ac_range_check_deny_cnt_reached, // IDs [163 +: 1]
      intr_racl_ctrl_racl_error, // IDs [162 +: 1]
      intr_mbx_pcie1_mbx_error, // IDs [161 +: 1]
      intr_mbx_pcie1_mbx_abort, // IDs [160 +: 1]
      intr_mbx_pcie1_mbx_ready, // IDs [159 +: 1]
      intr_mbx_pcie0_mbx_error, // IDs [158 +: 1]
      intr_mbx_pcie0_mbx_abort, // IDs [157 +: 1]
      intr_mbx_pcie0_mbx_ready, // IDs [156 +: 1]
      intr_mbx_jtag_mbx_error, // IDs [155 +: 1]
      intr_mbx_jtag_mbx_abort, // IDs [154 +: 1]
      intr_mbx_jtag_mbx_ready, // IDs [153 +: 1]
      intr_mbx6_mbx_error, // IDs [152 +: 1]
      intr_mbx6_mbx_abort, // IDs [151 +: 1]
      intr_mbx6_mbx_ready, // IDs [150 +: 1]
      intr_mbx5_mbx_error, // IDs [149 +: 1]
      intr_mbx5_mbx_abort, // IDs [148 +: 1]
      intr_mbx5_mbx_ready, // IDs [147 +: 1]
      intr_mbx4_mbx_error, // IDs [146 +: 1]
      intr_mbx4_mbx_abort, // IDs [145 +: 1]
      intr_mbx4_mbx_ready, // IDs [144 +: 1]
      intr_mbx3_mbx_error, // IDs [143 +: 1]
      intr_mbx3_mbx_abort, // IDs [142 +: 1]
      intr_mbx3_mbx_ready, // IDs [141 +: 1]
      intr_mbx2_mbx_error, // IDs [140 +: 1]
      intr_mbx2_mbx_abort, // IDs [139 +: 1]
      intr_mbx2_mbx_ready, // IDs [138 +: 1]
      intr_mbx1_mbx_error, // IDs [137 +: 1]
      intr_mbx1_mbx_abort, // IDs [136 +: 1]
      intr_mbx1_mbx_ready, // IDs [135 +: 1]
      intr_mbx0_mbx_error, // IDs [134 +: 1]
      intr_mbx0_mbx_abort, // IDs [133 +: 1]
      intr_mbx0_mbx_ready, // IDs [132 +: 1]
      intr_dma_dma_error, // IDs [131 +: 1]
      intr_dma_dma_chunk_done, // IDs [130 +: 1]
      intr_dma_dma_done, // IDs [129 +: 1]
      intr_edn1_edn_fatal_err, // IDs [128 +: 1]
      intr_edn1_edn_cmd_req_done, // IDs [127 +: 1]
      intr_edn0_edn_fatal_err, // IDs [126 +: 1]
      intr_edn0_edn_cmd_req_done, // IDs [125 +: 1]
      intr_entropy_src_es_fatal_err, // IDs [124 +: 1]
      intr_entropy_src_es_observe_fifo_ready, // IDs [123 +: 1]
      intr_entropy_src_es_health_test_failed, // IDs [122 +: 1]
      intr_entropy_src_es_entropy_valid, // IDs [121 +: 1]
      intr_csrng_cs_fatal_err, // IDs [120 +: 1]
      intr_csrng_cs_hw_inst_exc, // IDs [119 +: 1]
      intr_csrng_cs_entropy_req, // IDs [118 +: 1]
      intr_csrng_cs_cmd_req_done, // IDs [117 +: 1]
      intr_keymgr_dpe_op_done, // IDs [116 +: 1]
      intr_otbn_done, // IDs [115 +: 1]
      intr_kmac_kmac_err, // IDs [114 +: 1]
      intr_kmac_fifo_empty, // IDs [113 +: 1]
      intr_kmac_kmac_done, // IDs [112 +: 1]
      intr_hmac_hmac_err, // IDs [111 +: 1]
      intr_hmac_fifo_empty, // IDs [110 +: 1]
      intr_hmac_hmac_done, // IDs [109 +: 1]
      intr_soc_proxy_external, // IDs [77 +: 32]
      intr_aon_timer_aon_wdog_timer_bark, // IDs [76 +: 1]
      intr_aon_timer_aon_wkup_timer_expired, // IDs [75 +: 1]
      intr_pwrmgr_aon_wakeup, // IDs [74 +: 1]
      intr_spi_host0_spi_event, // IDs [73 +: 1]
      intr_spi_host0_error, // IDs [72 +: 1]
      intr_alert_handler_classd, // IDs [71 +: 1]
      intr_alert_handler_classc, // IDs [70 +: 1]
      intr_alert_handler_classb, // IDs [69 +: 1]
      intr_alert_handler_classa, // IDs [68 +: 1]
      intr_otp_ctrl_otp_error, // IDs [67 +: 1]
      intr_otp_ctrl_otp_operation_done, // IDs [66 +: 1]
      intr_rv_timer_timer_expired_hart0_timer0, // IDs [65 +: 1]
      intr_i2c0_host_timeout, // IDs [64 +: 1]
      intr_i2c0_unexp_stop, // IDs [63 +: 1]
      intr_i2c0_acq_stretch, // IDs [62 +: 1]
      intr_i2c0_tx_threshold, // IDs [61 +: 1]
      intr_i2c0_tx_stretch, // IDs [60 +: 1]
      intr_i2c0_cmd_complete, // IDs [59 +: 1]
      intr_i2c0_sda_unstable, // IDs [58 +: 1]
      intr_i2c0_stretch_timeout, // IDs [57 +: 1]
      intr_i2c0_sda_interference, // IDs [56 +: 1]
      intr_i2c0_scl_interference, // IDs [55 +: 1]
      intr_i2c0_controller_halt, // IDs [54 +: 1]
      intr_i2c0_rx_overflow, // IDs [53 +: 1]
      intr_i2c0_acq_threshold, // IDs [52 +: 1]
      intr_i2c0_rx_threshold, // IDs [51 +: 1]
      intr_i2c0_fmt_threshold, // IDs [50 +: 1]
      intr_spi_device_tpm_rdfifo_drop, // IDs [49 +: 1]
      intr_spi_device_tpm_rdfifo_cmd_end, // IDs [48 +: 1]
      intr_spi_device_tpm_header_not_empty, // IDs [47 +: 1]
      intr_spi_device_readbuf_flip, // IDs [46 +: 1]
      intr_spi_device_readbuf_watermark, // IDs [45 +: 1]
      intr_spi_device_upload_payload_overflow, // IDs [44 +: 1]
      intr_spi_device_upload_payload_not_empty, // IDs [43 +: 1]
      intr_spi_device_upload_cmdfifo_not_empty, // IDs [42 +: 1]
      intr_gpio_gpio, // IDs [10 +: 32]
      intr_uart0_tx_empty, // IDs [9 +: 1]
      intr_uart0_rx_parity_err, // IDs [8 +: 1]
      intr_uart0_rx_timeout, // IDs [7 +: 1]
      intr_uart0_rx_break_err, // IDs [6 +: 1]
      intr_uart0_rx_frame_err, // IDs [5 +: 1]
      intr_uart0_rx_overflow, // IDs [4 +: 1]
      intr_uart0_tx_done, // IDs [3 +: 1]
      intr_uart0_rx_watermark, // IDs [2 +: 1]
      intr_uart0_tx_watermark, // IDs [1 +: 1]
      1'b 0 // ID [0 +: 1] is a special case and tied to zero.
  };

  // TL-UL Crossbar
  xbar_main u_xbar_main (
    .clk_main_i (clkmgr_aon_clocks.clk_main_infra),
    .clk_fixed_i (clkmgr_aon_clocks.clk_io_div4_infra),
    .rst_main_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
    .rst_fixed_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),

    // port: tl_rv_core_ibex__corei
    .tl_rv_core_ibex__corei_i(main_tl_rv_core_ibex__corei_req),
    .tl_rv_core_ibex__corei_o(main_tl_rv_core_ibex__corei_rsp),

    // port: tl_rv_core_ibex__cored
    .tl_rv_core_ibex__cored_i(main_tl_rv_core_ibex__cored_req),
    .tl_rv_core_ibex__cored_o(main_tl_rv_core_ibex__cored_rsp),

    // port: tl_rv_dm__sba
    .tl_rv_dm__sba_i(main_tl_rv_dm__sba_req),
    .tl_rv_dm__sba_o(main_tl_rv_dm__sba_rsp),

    // port: tl_dma__host
    .tl_dma__host_i(main_tl_dma__host_req),
    .tl_dma__host_o(main_tl_dma__host_rsp),

    // port: tl_mbx0__sram
    .tl_mbx0__sram_i(main_tl_mbx0__sram_req),
    .tl_mbx0__sram_o(main_tl_mbx0__sram_rsp),

    // port: tl_mbx1__sram
    .tl_mbx1__sram_i(main_tl_mbx1__sram_req),
    .tl_mbx1__sram_o(main_tl_mbx1__sram_rsp),

    // port: tl_mbx2__sram
    .tl_mbx2__sram_i(main_tl_mbx2__sram_req),
    .tl_mbx2__sram_o(main_tl_mbx2__sram_rsp),

    // port: tl_mbx3__sram
    .tl_mbx3__sram_i(main_tl_mbx3__sram_req),
    .tl_mbx3__sram_o(main_tl_mbx3__sram_rsp),

    // port: tl_mbx4__sram
    .tl_mbx4__sram_i(main_tl_mbx4__sram_req),
    .tl_mbx4__sram_o(main_tl_mbx4__sram_rsp),

    // port: tl_mbx5__sram
    .tl_mbx5__sram_i(main_tl_mbx5__sram_req),
    .tl_mbx5__sram_o(main_tl_mbx5__sram_rsp),

    // port: tl_mbx6__sram
    .tl_mbx6__sram_i(main_tl_mbx6__sram_req),
    .tl_mbx6__sram_o(main_tl_mbx6__sram_rsp),

    // port: tl_mbx_jtag__sram
    .tl_mbx_jtag__sram_i(main_tl_mbx_jtag__sram_req),
    .tl_mbx_jtag__sram_o(main_tl_mbx_jtag__sram_rsp),

    // port: tl_mbx_pcie0__sram
    .tl_mbx_pcie0__sram_i(main_tl_mbx_pcie0__sram_req),
    .tl_mbx_pcie0__sram_o(main_tl_mbx_pcie0__sram_rsp),

    // port: tl_mbx_pcie1__sram
    .tl_mbx_pcie1__sram_i(main_tl_mbx_pcie1__sram_req),
    .tl_mbx_pcie1__sram_o(main_tl_mbx_pcie1__sram_rsp),

    // port: tl_rv_dm__regs
    .tl_rv_dm__regs_o(rv_dm_regs_tl_d_req),
    .tl_rv_dm__regs_i(rv_dm_regs_tl_d_rsp),

    // port: tl_rv_dm__mem
    .tl_rv_dm__mem_o(rv_dm_mem_tl_d_req),
    .tl_rv_dm__mem_i(rv_dm_mem_tl_d_rsp),

    // port: tl_rom_ctrl0__rom
    .tl_rom_ctrl0__rom_o(rom_ctrl0_rom_tl_req),
    .tl_rom_ctrl0__rom_i(rom_ctrl0_rom_tl_rsp),

    // port: tl_rom_ctrl0__regs
    .tl_rom_ctrl0__regs_o(rom_ctrl0_regs_tl_req),
    .tl_rom_ctrl0__regs_i(rom_ctrl0_regs_tl_rsp),

    // port: tl_rom_ctrl1__rom
    .tl_rom_ctrl1__rom_o(rom_ctrl1_rom_tl_req),
    .tl_rom_ctrl1__rom_i(rom_ctrl1_rom_tl_rsp),

    // port: tl_rom_ctrl1__regs
    .tl_rom_ctrl1__regs_o(rom_ctrl1_regs_tl_req),
    .tl_rom_ctrl1__regs_i(rom_ctrl1_regs_tl_rsp),

    // port: tl_peri
    .tl_peri_o(main_tl_peri_req),
    .tl_peri_i(main_tl_peri_rsp),

    // port: tl_soc_proxy__core
    .tl_soc_proxy__core_o(soc_proxy_core_tl_req),
    .tl_soc_proxy__core_i(soc_proxy_core_tl_rsp),

    // port: tl_soc_proxy__ctn
    .tl_soc_proxy__ctn_o(soc_proxy_ctn_tl_req),
    .tl_soc_proxy__ctn_i(soc_proxy_ctn_tl_rsp),

    // port: tl_hmac
    .tl_hmac_o(hmac_tl_req),
    .tl_hmac_i(hmac_tl_rsp),

    // port: tl_kmac
    .tl_kmac_o(kmac_tl_req),
    .tl_kmac_i(kmac_tl_rsp),

    // port: tl_aes
    .tl_aes_o(aes_tl_req),
    .tl_aes_i(aes_tl_rsp),

    // port: tl_entropy_src
    .tl_entropy_src_o(entropy_src_tl_req),
    .tl_entropy_src_i(entropy_src_tl_rsp),

    // port: tl_csrng
    .tl_csrng_o(csrng_tl_req),
    .tl_csrng_i(csrng_tl_rsp),

    // port: tl_edn0
    .tl_edn0_o(edn0_tl_req),
    .tl_edn0_i(edn0_tl_rsp),

    // port: tl_edn1
    .tl_edn1_o(edn1_tl_req),
    .tl_edn1_i(edn1_tl_rsp),

    // port: tl_rv_plic
    .tl_rv_plic_o(rv_plic_tl_req),
    .tl_rv_plic_i(rv_plic_tl_rsp),

    // port: tl_otbn
    .tl_otbn_o(otbn_tl_req),
    .tl_otbn_i(otbn_tl_rsp),

    // port: tl_keymgr_dpe
    .tl_keymgr_dpe_o(keymgr_dpe_tl_req),
    .tl_keymgr_dpe_i(keymgr_dpe_tl_rsp),

    // port: tl_rv_core_ibex__cfg
    .tl_rv_core_ibex__cfg_o(rv_core_ibex_cfg_tl_d_req),
    .tl_rv_core_ibex__cfg_i(rv_core_ibex_cfg_tl_d_rsp),

    // port: tl_sram_ctrl_main__regs
    .tl_sram_ctrl_main__regs_o(sram_ctrl_main_regs_tl_req),
    .tl_sram_ctrl_main__regs_i(sram_ctrl_main_regs_tl_rsp),

    // port: tl_sram_ctrl_main__ram
    .tl_sram_ctrl_main__ram_o(sram_ctrl_main_ram_tl_req),
    .tl_sram_ctrl_main__ram_i(sram_ctrl_main_ram_tl_rsp),

    // port: tl_sram_ctrl_mbox__regs
    .tl_sram_ctrl_mbox__regs_o(sram_ctrl_mbox_regs_tl_req),
    .tl_sram_ctrl_mbox__regs_i(sram_ctrl_mbox_regs_tl_rsp),

    // port: tl_sram_ctrl_mbox__ram
    .tl_sram_ctrl_mbox__ram_o(sram_ctrl_mbox_ram_tl_req),
    .tl_sram_ctrl_mbox__ram_i(sram_ctrl_mbox_ram_tl_rsp),

    // port: tl_dma
    .tl_dma_o(dma_tl_d_req),
    .tl_dma_i(dma_tl_d_rsp),

    // port: tl_mbx0__core
    .tl_mbx0__core_o(mbx0_core_tl_d_req),
    .tl_mbx0__core_i(mbx0_core_tl_d_rsp),

    // port: tl_mbx1__core
    .tl_mbx1__core_o(mbx1_core_tl_d_req),
    .tl_mbx1__core_i(mbx1_core_tl_d_rsp),

    // port: tl_mbx2__core
    .tl_mbx2__core_o(mbx2_core_tl_d_req),
    .tl_mbx2__core_i(mbx2_core_tl_d_rsp),

    // port: tl_mbx3__core
    .tl_mbx3__core_o(mbx3_core_tl_d_req),
    .tl_mbx3__core_i(mbx3_core_tl_d_rsp),

    // port: tl_mbx4__core
    .tl_mbx4__core_o(mbx4_core_tl_d_req),
    .tl_mbx4__core_i(mbx4_core_tl_d_rsp),

    // port: tl_mbx5__core
    .tl_mbx5__core_o(mbx5_core_tl_d_req),
    .tl_mbx5__core_i(mbx5_core_tl_d_rsp),

    // port: tl_mbx6__core
    .tl_mbx6__core_o(mbx6_core_tl_d_req),
    .tl_mbx6__core_i(mbx6_core_tl_d_rsp),

    // port: tl_mbx_jtag__core
    .tl_mbx_jtag__core_o(mbx_jtag_core_tl_d_req),
    .tl_mbx_jtag__core_i(mbx_jtag_core_tl_d_rsp),

    // port: tl_mbx_pcie0__core
    .tl_mbx_pcie0__core_o(mbx_pcie0_core_tl_d_req),
    .tl_mbx_pcie0__core_i(mbx_pcie0_core_tl_d_rsp),

    // port: tl_mbx_pcie1__core
    .tl_mbx_pcie1__core_o(mbx_pcie1_core_tl_d_req),
    .tl_mbx_pcie1__core_i(mbx_pcie1_core_tl_d_rsp),


    .scanmode_i
  );
  xbar_peri u_xbar_peri (
    .clk_peri_i (clkmgr_aon_clocks.clk_io_div4_infra),
    .rst_peri_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),

    // port: tl_main
    .tl_main_i(main_tl_peri_req),
    .tl_main_o(main_tl_peri_rsp),

    // port: tl_uart0
    .tl_uart0_o(uart0_tl_req),
    .tl_uart0_i(uart0_tl_rsp),

    // port: tl_i2c0
    .tl_i2c0_o(i2c0_tl_req),
    .tl_i2c0_i(i2c0_tl_rsp),

    // port: tl_gpio
    .tl_gpio_o(gpio_tl_req),
    .tl_gpio_i(gpio_tl_rsp),

    // port: tl_spi_host0
    .tl_spi_host0_o(spi_host0_tl_req),
    .tl_spi_host0_i(spi_host0_tl_rsp),

    // port: tl_spi_device
    .tl_spi_device_o(spi_device_tl_req),
    .tl_spi_device_i(spi_device_tl_rsp),

    // port: tl_rv_timer
    .tl_rv_timer_o(rv_timer_tl_req),
    .tl_rv_timer_i(rv_timer_tl_rsp),

    // port: tl_pwrmgr_aon
    .tl_pwrmgr_aon_o(pwrmgr_aon_tl_req),
    .tl_pwrmgr_aon_i(pwrmgr_aon_tl_rsp),

    // port: tl_rstmgr_aon
    .tl_rstmgr_aon_o(rstmgr_aon_tl_req),
    .tl_rstmgr_aon_i(rstmgr_aon_tl_rsp),

    // port: tl_clkmgr_aon
    .tl_clkmgr_aon_o(clkmgr_aon_tl_req),
    .tl_clkmgr_aon_i(clkmgr_aon_tl_rsp),

    // port: tl_pinmux_aon
    .tl_pinmux_aon_o(pinmux_aon_tl_req),
    .tl_pinmux_aon_i(pinmux_aon_tl_rsp),

    // port: tl_otp_ctrl__core
    .tl_otp_ctrl__core_o(otp_ctrl_core_tl_req),
    .tl_otp_ctrl__core_i(otp_ctrl_core_tl_rsp),

    // port: tl_otp_macro__prim
    .tl_otp_macro__prim_o(otp_macro_prim_tl_req),
    .tl_otp_macro__prim_i(otp_macro_prim_tl_rsp),

    // port: tl_lc_ctrl__regs
    .tl_lc_ctrl__regs_o(lc_ctrl_regs_tl_req),
    .tl_lc_ctrl__regs_i(lc_ctrl_regs_tl_rsp),

    // port: tl_alert_handler
    .tl_alert_handler_o(alert_handler_tl_req),
    .tl_alert_handler_i(alert_handler_tl_rsp),

    // port: tl_sram_ctrl_ret_aon__regs
    .tl_sram_ctrl_ret_aon__regs_o(sram_ctrl_ret_aon_regs_tl_req),
    .tl_sram_ctrl_ret_aon__regs_i(sram_ctrl_ret_aon_regs_tl_rsp),

    // port: tl_sram_ctrl_ret_aon__ram
    .tl_sram_ctrl_ret_aon__ram_o(sram_ctrl_ret_aon_ram_tl_req),
    .tl_sram_ctrl_ret_aon__ram_i(sram_ctrl_ret_aon_ram_tl_rsp),

    // port: tl_aon_timer_aon
    .tl_aon_timer_aon_o(aon_timer_aon_tl_req),
    .tl_aon_timer_aon_i(aon_timer_aon_tl_rsp),

    // port: tl_ast
    .tl_ast_o(ast_tl_req_o),
    .tl_ast_i(ast_tl_rsp_i),

    // port: tl_soc_dbg_ctrl__core
    .tl_soc_dbg_ctrl__core_o(soc_dbg_ctrl_core_tl_req),
    .tl_soc_dbg_ctrl__core_i(soc_dbg_ctrl_core_tl_rsp),


    .scanmode_i
  );
  xbar_mbx u_xbar_mbx (
    .clk_mbx_i (clkmgr_aon_clocks.clk_main_infra),
    .rst_mbx_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),

    // port: tl_mbx
    .tl_mbx_i(mbx_tl_req_i),
    .tl_mbx_o(mbx_tl_rsp_o),

    // port: tl_mbx0__soc
    .tl_mbx0__soc_o(mbx0_soc_tl_d_req),
    .tl_mbx0__soc_i(mbx0_soc_tl_d_rsp),

    // port: tl_mbx1__soc
    .tl_mbx1__soc_o(mbx1_soc_tl_d_req),
    .tl_mbx1__soc_i(mbx1_soc_tl_d_rsp),

    // port: tl_mbx2__soc
    .tl_mbx2__soc_o(mbx2_soc_tl_d_req),
    .tl_mbx2__soc_i(mbx2_soc_tl_d_rsp),

    // port: tl_mbx3__soc
    .tl_mbx3__soc_o(mbx3_soc_tl_d_req),
    .tl_mbx3__soc_i(mbx3_soc_tl_d_rsp),

    // port: tl_mbx4__soc
    .tl_mbx4__soc_o(mbx4_soc_tl_d_req),
    .tl_mbx4__soc_i(mbx4_soc_tl_d_rsp),

    // port: tl_mbx5__soc
    .tl_mbx5__soc_o(mbx5_soc_tl_d_req),
    .tl_mbx5__soc_i(mbx5_soc_tl_d_rsp),

    // port: tl_mbx6__soc
    .tl_mbx6__soc_o(mbx6_soc_tl_d_req),
    .tl_mbx6__soc_i(mbx6_soc_tl_d_rsp),

    // port: tl_mbx_pcie0__soc
    .tl_mbx_pcie0__soc_o(mbx_pcie0_soc_tl_d_req),
    .tl_mbx_pcie0__soc_i(mbx_pcie0_soc_tl_d_rsp),

    // port: tl_mbx_pcie1__soc
    .tl_mbx_pcie1__soc_o(mbx_pcie1_soc_tl_d_req),
    .tl_mbx_pcie1__soc_i(mbx_pcie1_soc_tl_d_rsp),

    // port: tl_racl_ctrl
    .tl_racl_ctrl_o(racl_ctrl_tl_req),
    .tl_racl_ctrl_i(racl_ctrl_tl_rsp),

    // port: tl_ac_range_check
    .tl_ac_range_check_o(ac_range_check_tl_req),
    .tl_ac_range_check_i(ac_range_check_tl_rsp),


    .scanmode_i
  );
  xbar_dbg u_xbar_dbg (
    .clk_dbg_i (clkmgr_aon_clocks.clk_main_infra),
    .clk_peri_i (clkmgr_aon_clocks.clk_io_div4_infra),
    .rst_dbg_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
    .rst_peri_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),

    // port: tl_dbg
    .tl_dbg_i(dbg_tl_req_i),
    .tl_dbg_o(dbg_tl_rsp_o),

    // port: tl_rv_dm__dbg
    .tl_rv_dm__dbg_o(rv_dm_dbg_tl_d_req),
    .tl_rv_dm__dbg_i(rv_dm_dbg_tl_d_rsp),

    // port: tl_mbx_jtag__soc
    .tl_mbx_jtag__soc_o(mbx_jtag_soc_tl_d_req),
    .tl_mbx_jtag__soc_i(mbx_jtag_soc_tl_d_rsp),

    // port: tl_lc_ctrl__dmi
    .tl_lc_ctrl__dmi_o(lc_ctrl_dmi_tl_req),
    .tl_lc_ctrl__dmi_i(lc_ctrl_dmi_tl_rsp),

    // port: tl_soc_dbg_ctrl__jtag
    .tl_soc_dbg_ctrl__jtag_o(soc_dbg_ctrl_jtag_tl_req),
    .tl_soc_dbg_ctrl__jtag_i(soc_dbg_ctrl_jtag_tl_rsp),


    .scanmode_i
  );

  // Pinmux connections
  // All muxed inputs
  assign cio_soc_proxy_soc_gpi_p2d[12] = mio_p2d[MioInSocProxySocGpi12];
  assign cio_soc_proxy_soc_gpi_p2d[13] = mio_p2d[MioInSocProxySocGpi13];
  assign cio_soc_proxy_soc_gpi_p2d[14] = mio_p2d[MioInSocProxySocGpi14];
  assign cio_soc_proxy_soc_gpi_p2d[15] = mio_p2d[MioInSocProxySocGpi15];

  // All muxed outputs
  assign mio_d2p[MioOutSocProxySocGpo12] = cio_soc_proxy_soc_gpo_d2p[12];
  assign mio_d2p[MioOutSocProxySocGpo13] = cio_soc_proxy_soc_gpo_d2p[13];
  assign mio_d2p[MioOutSocProxySocGpo14] = cio_soc_proxy_soc_gpo_d2p[14];
  assign mio_d2p[MioOutSocProxySocGpo15] = cio_soc_proxy_soc_gpo_d2p[15];
  assign mio_d2p[MioOutOtpMacroTest0] = cio_otp_macro_test_d2p[0];

  // All muxed output enables
  assign mio_en_d2p[MioOutSocProxySocGpo12] = cio_soc_proxy_soc_gpo_en_d2p[12];
  assign mio_en_d2p[MioOutSocProxySocGpo13] = cio_soc_proxy_soc_gpo_en_d2p[13];
  assign mio_en_d2p[MioOutSocProxySocGpo14] = cio_soc_proxy_soc_gpo_en_d2p[14];
  assign mio_en_d2p[MioOutSocProxySocGpo15] = cio_soc_proxy_soc_gpo_en_d2p[15];
  assign mio_en_d2p[MioOutOtpMacroTest0] = cio_otp_macro_test_en_d2p[0];

  // All dedicated inputs
  logic [72:0] unused_dio_p2d;
  assign unused_dio_p2d = dio_p2d;
  assign cio_spi_host0_sd_p2d[0] = dio_p2d[DioSpiHost0Sd0];
  assign cio_spi_host0_sd_p2d[1] = dio_p2d[DioSpiHost0Sd1];
  assign cio_spi_host0_sd_p2d[2] = dio_p2d[DioSpiHost0Sd2];
  assign cio_spi_host0_sd_p2d[3] = dio_p2d[DioSpiHost0Sd3];
  assign cio_spi_device_sd_p2d[0] = dio_p2d[DioSpiDeviceSd0];
  assign cio_spi_device_sd_p2d[1] = dio_p2d[DioSpiDeviceSd1];
  assign cio_spi_device_sd_p2d[2] = dio_p2d[DioSpiDeviceSd2];
  assign cio_spi_device_sd_p2d[3] = dio_p2d[DioSpiDeviceSd3];
  assign cio_i2c0_scl_p2d = dio_p2d[DioI2c0Scl];
  assign cio_i2c0_sda_p2d = dio_p2d[DioI2c0Sda];
  assign cio_gpio_gpio_p2d[0] = dio_p2d[DioGpioGpio0];
  assign cio_gpio_gpio_p2d[1] = dio_p2d[DioGpioGpio1];
  assign cio_gpio_gpio_p2d[2] = dio_p2d[DioGpioGpio2];
  assign cio_gpio_gpio_p2d[3] = dio_p2d[DioGpioGpio3];
  assign cio_gpio_gpio_p2d[4] = dio_p2d[DioGpioGpio4];
  assign cio_gpio_gpio_p2d[5] = dio_p2d[DioGpioGpio5];
  assign cio_gpio_gpio_p2d[6] = dio_p2d[DioGpioGpio6];
  assign cio_gpio_gpio_p2d[7] = dio_p2d[DioGpioGpio7];
  assign cio_gpio_gpio_p2d[8] = dio_p2d[DioGpioGpio8];
  assign cio_gpio_gpio_p2d[9] = dio_p2d[DioGpioGpio9];
  assign cio_gpio_gpio_p2d[10] = dio_p2d[DioGpioGpio10];
  assign cio_gpio_gpio_p2d[11] = dio_p2d[DioGpioGpio11];
  assign cio_gpio_gpio_p2d[12] = dio_p2d[DioGpioGpio12];
  assign cio_gpio_gpio_p2d[13] = dio_p2d[DioGpioGpio13];
  assign cio_gpio_gpio_p2d[14] = dio_p2d[DioGpioGpio14];
  assign cio_gpio_gpio_p2d[15] = dio_p2d[DioGpioGpio15];
  assign cio_gpio_gpio_p2d[16] = dio_p2d[DioGpioGpio16];
  assign cio_gpio_gpio_p2d[17] = dio_p2d[DioGpioGpio17];
  assign cio_gpio_gpio_p2d[18] = dio_p2d[DioGpioGpio18];
  assign cio_gpio_gpio_p2d[19] = dio_p2d[DioGpioGpio19];
  assign cio_gpio_gpio_p2d[20] = dio_p2d[DioGpioGpio20];
  assign cio_gpio_gpio_p2d[21] = dio_p2d[DioGpioGpio21];
  assign cio_gpio_gpio_p2d[22] = dio_p2d[DioGpioGpio22];
  assign cio_gpio_gpio_p2d[23] = dio_p2d[DioGpioGpio23];
  assign cio_gpio_gpio_p2d[24] = dio_p2d[DioGpioGpio24];
  assign cio_gpio_gpio_p2d[25] = dio_p2d[DioGpioGpio25];
  assign cio_gpio_gpio_p2d[26] = dio_p2d[DioGpioGpio26];
  assign cio_gpio_gpio_p2d[27] = dio_p2d[DioGpioGpio27];
  assign cio_gpio_gpio_p2d[28] = dio_p2d[DioGpioGpio28];
  assign cio_gpio_gpio_p2d[29] = dio_p2d[DioGpioGpio29];
  assign cio_gpio_gpio_p2d[30] = dio_p2d[DioGpioGpio30];
  assign cio_gpio_gpio_p2d[31] = dio_p2d[DioGpioGpio31];
  assign cio_spi_device_sck_p2d = dio_p2d[DioSpiDeviceSck];
  assign cio_spi_device_csb_p2d = dio_p2d[DioSpiDeviceCsb];
  assign cio_spi_device_tpm_csb_p2d = dio_p2d[DioSpiDeviceTpmCsb];
  assign cio_uart0_rx_p2d = dio_p2d[DioUart0Rx];
  assign cio_soc_proxy_soc_gpi_p2d[0] = dio_p2d[DioSocProxySocGpi0];
  assign cio_soc_proxy_soc_gpi_p2d[1] = dio_p2d[DioSocProxySocGpi1];
  assign cio_soc_proxy_soc_gpi_p2d[2] = dio_p2d[DioSocProxySocGpi2];
  assign cio_soc_proxy_soc_gpi_p2d[3] = dio_p2d[DioSocProxySocGpi3];
  assign cio_soc_proxy_soc_gpi_p2d[4] = dio_p2d[DioSocProxySocGpi4];
  assign cio_soc_proxy_soc_gpi_p2d[5] = dio_p2d[DioSocProxySocGpi5];
  assign cio_soc_proxy_soc_gpi_p2d[6] = dio_p2d[DioSocProxySocGpi6];
  assign cio_soc_proxy_soc_gpi_p2d[7] = dio_p2d[DioSocProxySocGpi7];
  assign cio_soc_proxy_soc_gpi_p2d[8] = dio_p2d[DioSocProxySocGpi8];
  assign cio_soc_proxy_soc_gpi_p2d[9] = dio_p2d[DioSocProxySocGpi9];
  assign cio_soc_proxy_soc_gpi_p2d[10] = dio_p2d[DioSocProxySocGpi10];
  assign cio_soc_proxy_soc_gpi_p2d[11] = dio_p2d[DioSocProxySocGpi11];

    // All dedicated outputs
  assign dio_d2p[DioSpiHost0Sd0] = cio_spi_host0_sd_d2p[0];
  assign dio_d2p[DioSpiHost0Sd1] = cio_spi_host0_sd_d2p[1];
  assign dio_d2p[DioSpiHost0Sd2] = cio_spi_host0_sd_d2p[2];
  assign dio_d2p[DioSpiHost0Sd3] = cio_spi_host0_sd_d2p[3];
  assign dio_d2p[DioSpiDeviceSd0] = cio_spi_device_sd_d2p[0];
  assign dio_d2p[DioSpiDeviceSd1] = cio_spi_device_sd_d2p[1];
  assign dio_d2p[DioSpiDeviceSd2] = cio_spi_device_sd_d2p[2];
  assign dio_d2p[DioSpiDeviceSd3] = cio_spi_device_sd_d2p[3];
  assign dio_d2p[DioI2c0Scl] = cio_i2c0_scl_d2p;
  assign dio_d2p[DioI2c0Sda] = cio_i2c0_sda_d2p;
  assign dio_d2p[DioGpioGpio0] = cio_gpio_gpio_d2p[0];
  assign dio_d2p[DioGpioGpio1] = cio_gpio_gpio_d2p[1];
  assign dio_d2p[DioGpioGpio2] = cio_gpio_gpio_d2p[2];
  assign dio_d2p[DioGpioGpio3] = cio_gpio_gpio_d2p[3];
  assign dio_d2p[DioGpioGpio4] = cio_gpio_gpio_d2p[4];
  assign dio_d2p[DioGpioGpio5] = cio_gpio_gpio_d2p[5];
  assign dio_d2p[DioGpioGpio6] = cio_gpio_gpio_d2p[6];
  assign dio_d2p[DioGpioGpio7] = cio_gpio_gpio_d2p[7];
  assign dio_d2p[DioGpioGpio8] = cio_gpio_gpio_d2p[8];
  assign dio_d2p[DioGpioGpio9] = cio_gpio_gpio_d2p[9];
  assign dio_d2p[DioGpioGpio10] = cio_gpio_gpio_d2p[10];
  assign dio_d2p[DioGpioGpio11] = cio_gpio_gpio_d2p[11];
  assign dio_d2p[DioGpioGpio12] = cio_gpio_gpio_d2p[12];
  assign dio_d2p[DioGpioGpio13] = cio_gpio_gpio_d2p[13];
  assign dio_d2p[DioGpioGpio14] = cio_gpio_gpio_d2p[14];
  assign dio_d2p[DioGpioGpio15] = cio_gpio_gpio_d2p[15];
  assign dio_d2p[DioGpioGpio16] = cio_gpio_gpio_d2p[16];
  assign dio_d2p[DioGpioGpio17] = cio_gpio_gpio_d2p[17];
  assign dio_d2p[DioGpioGpio18] = cio_gpio_gpio_d2p[18];
  assign dio_d2p[DioGpioGpio19] = cio_gpio_gpio_d2p[19];
  assign dio_d2p[DioGpioGpio20] = cio_gpio_gpio_d2p[20];
  assign dio_d2p[DioGpioGpio21] = cio_gpio_gpio_d2p[21];
  assign dio_d2p[DioGpioGpio22] = cio_gpio_gpio_d2p[22];
  assign dio_d2p[DioGpioGpio23] = cio_gpio_gpio_d2p[23];
  assign dio_d2p[DioGpioGpio24] = cio_gpio_gpio_d2p[24];
  assign dio_d2p[DioGpioGpio25] = cio_gpio_gpio_d2p[25];
  assign dio_d2p[DioGpioGpio26] = cio_gpio_gpio_d2p[26];
  assign dio_d2p[DioGpioGpio27] = cio_gpio_gpio_d2p[27];
  assign dio_d2p[DioGpioGpio28] = cio_gpio_gpio_d2p[28];
  assign dio_d2p[DioGpioGpio29] = cio_gpio_gpio_d2p[29];
  assign dio_d2p[DioGpioGpio30] = cio_gpio_gpio_d2p[30];
  assign dio_d2p[DioGpioGpio31] = cio_gpio_gpio_d2p[31];
  assign dio_d2p[DioSpiDeviceSck] = 1'b0;
  assign dio_d2p[DioSpiDeviceCsb] = 1'b0;
  assign dio_d2p[DioSpiDeviceTpmCsb] = 1'b0;
  assign dio_d2p[DioUart0Rx] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi0] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi1] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi2] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi3] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi4] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi5] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi6] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi7] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi8] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi9] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi10] = 1'b0;
  assign dio_d2p[DioSocProxySocGpi11] = 1'b0;
  assign dio_d2p[DioSpiHost0Sck] = cio_spi_host0_sck_d2p;
  assign dio_d2p[DioSpiHost0Csb] = cio_spi_host0_csb_d2p;
  assign dio_d2p[DioUart0Tx] = cio_uart0_tx_d2p;
  assign dio_d2p[DioSocProxySocGpo0] = cio_soc_proxy_soc_gpo_d2p[0];
  assign dio_d2p[DioSocProxySocGpo1] = cio_soc_proxy_soc_gpo_d2p[1];
  assign dio_d2p[DioSocProxySocGpo2] = cio_soc_proxy_soc_gpo_d2p[2];
  assign dio_d2p[DioSocProxySocGpo3] = cio_soc_proxy_soc_gpo_d2p[3];
  assign dio_d2p[DioSocProxySocGpo4] = cio_soc_proxy_soc_gpo_d2p[4];
  assign dio_d2p[DioSocProxySocGpo5] = cio_soc_proxy_soc_gpo_d2p[5];
  assign dio_d2p[DioSocProxySocGpo6] = cio_soc_proxy_soc_gpo_d2p[6];
  assign dio_d2p[DioSocProxySocGpo7] = cio_soc_proxy_soc_gpo_d2p[7];
  assign dio_d2p[DioSocProxySocGpo8] = cio_soc_proxy_soc_gpo_d2p[8];
  assign dio_d2p[DioSocProxySocGpo9] = cio_soc_proxy_soc_gpo_d2p[9];
  assign dio_d2p[DioSocProxySocGpo10] = cio_soc_proxy_soc_gpo_d2p[10];
  assign dio_d2p[DioSocProxySocGpo11] = cio_soc_proxy_soc_gpo_d2p[11];

  // All dedicated output enables
  assign dio_en_d2p[DioSpiHost0Sd0] = cio_spi_host0_sd_en_d2p[0];
  assign dio_en_d2p[DioSpiHost0Sd1] = cio_spi_host0_sd_en_d2p[1];
  assign dio_en_d2p[DioSpiHost0Sd2] = cio_spi_host0_sd_en_d2p[2];
  assign dio_en_d2p[DioSpiHost0Sd3] = cio_spi_host0_sd_en_d2p[3];
  assign dio_en_d2p[DioSpiDeviceSd0] = cio_spi_device_sd_en_d2p[0];
  assign dio_en_d2p[DioSpiDeviceSd1] = cio_spi_device_sd_en_d2p[1];
  assign dio_en_d2p[DioSpiDeviceSd2] = cio_spi_device_sd_en_d2p[2];
  assign dio_en_d2p[DioSpiDeviceSd3] = cio_spi_device_sd_en_d2p[3];
  assign dio_en_d2p[DioI2c0Scl] = cio_i2c0_scl_en_d2p;
  assign dio_en_d2p[DioI2c0Sda] = cio_i2c0_sda_en_d2p;
  assign dio_en_d2p[DioGpioGpio0] = cio_gpio_gpio_en_d2p[0];
  assign dio_en_d2p[DioGpioGpio1] = cio_gpio_gpio_en_d2p[1];
  assign dio_en_d2p[DioGpioGpio2] = cio_gpio_gpio_en_d2p[2];
  assign dio_en_d2p[DioGpioGpio3] = cio_gpio_gpio_en_d2p[3];
  assign dio_en_d2p[DioGpioGpio4] = cio_gpio_gpio_en_d2p[4];
  assign dio_en_d2p[DioGpioGpio5] = cio_gpio_gpio_en_d2p[5];
  assign dio_en_d2p[DioGpioGpio6] = cio_gpio_gpio_en_d2p[6];
  assign dio_en_d2p[DioGpioGpio7] = cio_gpio_gpio_en_d2p[7];
  assign dio_en_d2p[DioGpioGpio8] = cio_gpio_gpio_en_d2p[8];
  assign dio_en_d2p[DioGpioGpio9] = cio_gpio_gpio_en_d2p[9];
  assign dio_en_d2p[DioGpioGpio10] = cio_gpio_gpio_en_d2p[10];
  assign dio_en_d2p[DioGpioGpio11] = cio_gpio_gpio_en_d2p[11];
  assign dio_en_d2p[DioGpioGpio12] = cio_gpio_gpio_en_d2p[12];
  assign dio_en_d2p[DioGpioGpio13] = cio_gpio_gpio_en_d2p[13];
  assign dio_en_d2p[DioGpioGpio14] = cio_gpio_gpio_en_d2p[14];
  assign dio_en_d2p[DioGpioGpio15] = cio_gpio_gpio_en_d2p[15];
  assign dio_en_d2p[DioGpioGpio16] = cio_gpio_gpio_en_d2p[16];
  assign dio_en_d2p[DioGpioGpio17] = cio_gpio_gpio_en_d2p[17];
  assign dio_en_d2p[DioGpioGpio18] = cio_gpio_gpio_en_d2p[18];
  assign dio_en_d2p[DioGpioGpio19] = cio_gpio_gpio_en_d2p[19];
  assign dio_en_d2p[DioGpioGpio20] = cio_gpio_gpio_en_d2p[20];
  assign dio_en_d2p[DioGpioGpio21] = cio_gpio_gpio_en_d2p[21];
  assign dio_en_d2p[DioGpioGpio22] = cio_gpio_gpio_en_d2p[22];
  assign dio_en_d2p[DioGpioGpio23] = cio_gpio_gpio_en_d2p[23];
  assign dio_en_d2p[DioGpioGpio24] = cio_gpio_gpio_en_d2p[24];
  assign dio_en_d2p[DioGpioGpio25] = cio_gpio_gpio_en_d2p[25];
  assign dio_en_d2p[DioGpioGpio26] = cio_gpio_gpio_en_d2p[26];
  assign dio_en_d2p[DioGpioGpio27] = cio_gpio_gpio_en_d2p[27];
  assign dio_en_d2p[DioGpioGpio28] = cio_gpio_gpio_en_d2p[28];
  assign dio_en_d2p[DioGpioGpio29] = cio_gpio_gpio_en_d2p[29];
  assign dio_en_d2p[DioGpioGpio30] = cio_gpio_gpio_en_d2p[30];
  assign dio_en_d2p[DioGpioGpio31] = cio_gpio_gpio_en_d2p[31];
  assign dio_en_d2p[DioSpiDeviceSck] = 1'b0;
  assign dio_en_d2p[DioSpiDeviceCsb] = 1'b0;
  assign dio_en_d2p[DioSpiDeviceTpmCsb] = 1'b0;
  assign dio_en_d2p[DioUart0Rx] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi0] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi1] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi2] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi3] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi4] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi5] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi6] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi7] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi8] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi9] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi10] = 1'b0;
  assign dio_en_d2p[DioSocProxySocGpi11] = 1'b0;
  assign dio_en_d2p[DioSpiHost0Sck] = cio_spi_host0_sck_en_d2p;
  assign dio_en_d2p[DioSpiHost0Csb] = cio_spi_host0_csb_en_d2p;
  assign dio_en_d2p[DioUart0Tx] = cio_uart0_tx_en_d2p;
  assign dio_en_d2p[DioSocProxySocGpo0] = cio_soc_proxy_soc_gpo_en_d2p[0];
  assign dio_en_d2p[DioSocProxySocGpo1] = cio_soc_proxy_soc_gpo_en_d2p[1];
  assign dio_en_d2p[DioSocProxySocGpo2] = cio_soc_proxy_soc_gpo_en_d2p[2];
  assign dio_en_d2p[DioSocProxySocGpo3] = cio_soc_proxy_soc_gpo_en_d2p[3];
  assign dio_en_d2p[DioSocProxySocGpo4] = cio_soc_proxy_soc_gpo_en_d2p[4];
  assign dio_en_d2p[DioSocProxySocGpo5] = cio_soc_proxy_soc_gpo_en_d2p[5];
  assign dio_en_d2p[DioSocProxySocGpo6] = cio_soc_proxy_soc_gpo_en_d2p[6];
  assign dio_en_d2p[DioSocProxySocGpo7] = cio_soc_proxy_soc_gpo_en_d2p[7];
  assign dio_en_d2p[DioSocProxySocGpo8] = cio_soc_proxy_soc_gpo_en_d2p[8];
  assign dio_en_d2p[DioSocProxySocGpo9] = cio_soc_proxy_soc_gpo_en_d2p[9];
  assign dio_en_d2p[DioSocProxySocGpo10] = cio_soc_proxy_soc_gpo_en_d2p[10];
  assign dio_en_d2p[DioSocProxySocGpo11] = cio_soc_proxy_soc_gpo_en_d2p[11];


  // make sure scanmode_i is never X (including during reset)
  `ASSERT_KNOWN(scanmodeKnown, scanmode_i, clk_main_i, 0)

  // TODO(#26288) : EnCsrngSwAppReadSize should not be present in Darjeeling; presently, this signal
  // must be used to avoid a lint error.
  logic unused_en_csrng;
  assign unused_en_csrng = ^otp_ctrl_otp_broadcast.hw_cfg1_data.en_csrng_sw_app_read;

endmodule
