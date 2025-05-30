// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// xbar_main module generated by `tlgen.py` tool
// all reset signals should be generated from one reset signal to not make any deadlock
//
// Interconnect
// rv_core_ibex.corei
//   -> s1n_13
//     -> sm1_14
//       -> rom_ctrl.rom
//     -> sm1_15
//       -> sram_ctrl_main.ram
//     -> sm1_16
//       -> flash_ctrl.mem
// rv_core_ibex.cored
//   -> s1n_17
//     -> sm1_14
//       -> rom_ctrl.rom
//     -> rom_ctrl.regs
//     -> sm1_15
//       -> sram_ctrl_main.ram
//     -> sm1_16
//       -> flash_ctrl.mem
//     -> asf_18
//       -> peri
//     -> flash_ctrl.core
//     -> flash_ctrl.prim
//     -> aes
//     -> rv_plic
//     -> sram_ctrl_main.regs
//     -> rv_core_ibex.cfg

module xbar_main (
  input clk_main_i,
  input clk_fixed_i,
  input rst_main_ni,
  input rst_fixed_ni,

  // Host interfaces
  input  tlul_pkg::tl_h2d_t tl_rv_core_ibex__corei_i,
  output tlul_pkg::tl_d2h_t tl_rv_core_ibex__corei_o,
  input  tlul_pkg::tl_h2d_t tl_rv_core_ibex__cored_i,
  output tlul_pkg::tl_d2h_t tl_rv_core_ibex__cored_o,

  // Device interfaces
  output tlul_pkg::tl_h2d_t tl_rom_ctrl__rom_o,
  input  tlul_pkg::tl_d2h_t tl_rom_ctrl__rom_i,
  output tlul_pkg::tl_h2d_t tl_rom_ctrl__regs_o,
  input  tlul_pkg::tl_d2h_t tl_rom_ctrl__regs_i,
  output tlul_pkg::tl_h2d_t tl_peri_o,
  input  tlul_pkg::tl_d2h_t tl_peri_i,
  output tlul_pkg::tl_h2d_t tl_flash_ctrl__core_o,
  input  tlul_pkg::tl_d2h_t tl_flash_ctrl__core_i,
  output tlul_pkg::tl_h2d_t tl_flash_ctrl__prim_o,
  input  tlul_pkg::tl_d2h_t tl_flash_ctrl__prim_i,
  output tlul_pkg::tl_h2d_t tl_flash_ctrl__mem_o,
  input  tlul_pkg::tl_d2h_t tl_flash_ctrl__mem_i,
  output tlul_pkg::tl_h2d_t tl_aes_o,
  input  tlul_pkg::tl_d2h_t tl_aes_i,
  output tlul_pkg::tl_h2d_t tl_rv_plic_o,
  input  tlul_pkg::tl_d2h_t tl_rv_plic_i,
  output tlul_pkg::tl_h2d_t tl_rv_core_ibex__cfg_o,
  input  tlul_pkg::tl_d2h_t tl_rv_core_ibex__cfg_i,
  output tlul_pkg::tl_h2d_t tl_sram_ctrl_main__regs_o,
  input  tlul_pkg::tl_d2h_t tl_sram_ctrl_main__regs_i,
  output tlul_pkg::tl_h2d_t tl_sram_ctrl_main__ram_o,
  input  tlul_pkg::tl_d2h_t tl_sram_ctrl_main__ram_i,

  input prim_mubi_pkg::mubi4_t scanmode_i
);

  import tlul_pkg::*;
  import tl_main_pkg::*;

  // scanmode_i is currently not used, but provisioned for future use
  // this assignment prevents lint warnings
  logic unused_scanmode;
  assign unused_scanmode = ^scanmode_i;

  tl_h2d_t tl_s1n_13_us_h2d ;
  tl_d2h_t tl_s1n_13_us_d2h ;


  tl_h2d_t tl_s1n_13_ds_h2d [3];
  tl_d2h_t tl_s1n_13_ds_d2h [3];

  // Create steering signal
  logic [1:0] dev_sel_s1n_13;


  tl_h2d_t tl_sm1_14_us_h2d [2];
  tl_d2h_t tl_sm1_14_us_d2h [2];

  tl_h2d_t tl_sm1_14_ds_h2d ;
  tl_d2h_t tl_sm1_14_ds_d2h ;


  tl_h2d_t tl_sm1_15_us_h2d [2];
  tl_d2h_t tl_sm1_15_us_d2h [2];

  tl_h2d_t tl_sm1_15_ds_h2d ;
  tl_d2h_t tl_sm1_15_ds_d2h ;


  tl_h2d_t tl_sm1_16_us_h2d [2];
  tl_d2h_t tl_sm1_16_us_d2h [2];

  tl_h2d_t tl_sm1_16_ds_h2d ;
  tl_d2h_t tl_sm1_16_ds_d2h ;

  tl_h2d_t tl_s1n_17_us_h2d ;
  tl_d2h_t tl_s1n_17_us_d2h ;


  tl_h2d_t tl_s1n_17_ds_h2d [11];
  tl_d2h_t tl_s1n_17_ds_d2h [11];

  // Create steering signal
  logic [3:0] dev_sel_s1n_17;

  tl_h2d_t tl_asf_18_us_h2d ;
  tl_d2h_t tl_asf_18_us_d2h ;
  tl_h2d_t tl_asf_18_ds_h2d ;
  tl_d2h_t tl_asf_18_ds_d2h ;



  assign tl_sm1_14_us_h2d[0] = tl_s1n_13_ds_h2d[0];
  assign tl_s1n_13_ds_d2h[0] = tl_sm1_14_us_d2h[0];

  assign tl_sm1_15_us_h2d[0] = tl_s1n_13_ds_h2d[1];
  assign tl_s1n_13_ds_d2h[1] = tl_sm1_15_us_d2h[0];

  assign tl_sm1_16_us_h2d[0] = tl_s1n_13_ds_h2d[2];
  assign tl_s1n_13_ds_d2h[2] = tl_sm1_16_us_d2h[0];

  assign tl_sm1_14_us_h2d[1] = tl_s1n_17_ds_h2d[0];
  assign tl_s1n_17_ds_d2h[0] = tl_sm1_14_us_d2h[1];

  assign tl_rom_ctrl__regs_o = tl_s1n_17_ds_h2d[1];
  assign tl_s1n_17_ds_d2h[1] = tl_rom_ctrl__regs_i;

  assign tl_sm1_15_us_h2d[1] = tl_s1n_17_ds_h2d[2];
  assign tl_s1n_17_ds_d2h[2] = tl_sm1_15_us_d2h[1];

  assign tl_sm1_16_us_h2d[1] = tl_s1n_17_ds_h2d[3];
  assign tl_s1n_17_ds_d2h[3] = tl_sm1_16_us_d2h[1];

  assign tl_asf_18_us_h2d = tl_s1n_17_ds_h2d[4];
  assign tl_s1n_17_ds_d2h[4] = tl_asf_18_us_d2h;

  assign tl_flash_ctrl__core_o = tl_s1n_17_ds_h2d[5];
  assign tl_s1n_17_ds_d2h[5] = tl_flash_ctrl__core_i;

  assign tl_flash_ctrl__prim_o = tl_s1n_17_ds_h2d[6];
  assign tl_s1n_17_ds_d2h[6] = tl_flash_ctrl__prim_i;

  assign tl_aes_o = tl_s1n_17_ds_h2d[7];
  assign tl_s1n_17_ds_d2h[7] = tl_aes_i;

  assign tl_rv_plic_o = tl_s1n_17_ds_h2d[8];
  assign tl_s1n_17_ds_d2h[8] = tl_rv_plic_i;

  assign tl_sram_ctrl_main__regs_o = tl_s1n_17_ds_h2d[9];
  assign tl_s1n_17_ds_d2h[9] = tl_sram_ctrl_main__regs_i;

  assign tl_rv_core_ibex__cfg_o = tl_s1n_17_ds_h2d[10];
  assign tl_s1n_17_ds_d2h[10] = tl_rv_core_ibex__cfg_i;

  assign tl_s1n_13_us_h2d = tl_rv_core_ibex__corei_i;
  assign tl_rv_core_ibex__corei_o = tl_s1n_13_us_d2h;

  assign tl_rom_ctrl__rom_o = tl_sm1_14_ds_h2d;
  assign tl_sm1_14_ds_d2h = tl_rom_ctrl__rom_i;

  assign tl_sram_ctrl_main__ram_o = tl_sm1_15_ds_h2d;
  assign tl_sm1_15_ds_d2h = tl_sram_ctrl_main__ram_i;

  assign tl_flash_ctrl__mem_o = tl_sm1_16_ds_h2d;
  assign tl_sm1_16_ds_d2h = tl_flash_ctrl__mem_i;

  assign tl_s1n_17_us_h2d = tl_rv_core_ibex__cored_i;
  assign tl_rv_core_ibex__cored_o = tl_s1n_17_us_d2h;

  assign tl_peri_o = tl_asf_18_ds_h2d;
  assign tl_asf_18_ds_d2h = tl_peri_i;

  always_comb begin
    // default steering to generate error response if address is not within the range
    dev_sel_s1n_13 = 2'd3;
    if ((tl_s1n_13_us_h2d.a_address &
         ~(ADDR_MASK_ROM_CTRL__ROM)) == ADDR_SPACE_ROM_CTRL__ROM) begin
      dev_sel_s1n_13 = 2'd0;

    end else if ((tl_s1n_13_us_h2d.a_address &
                  ~(ADDR_MASK_SRAM_CTRL_MAIN__RAM)) == ADDR_SPACE_SRAM_CTRL_MAIN__RAM) begin
      dev_sel_s1n_13 = 2'd1;

    end else if ((tl_s1n_13_us_h2d.a_address &
                  ~(ADDR_MASK_FLASH_CTRL__MEM)) == ADDR_SPACE_FLASH_CTRL__MEM) begin
      dev_sel_s1n_13 = 2'd2;
end
  end

  always_comb begin
    // default steering to generate error response if address is not within the range
    dev_sel_s1n_17 = 4'd11;
    if ((tl_s1n_17_us_h2d.a_address &
         ~(ADDR_MASK_ROM_CTRL__ROM)) == ADDR_SPACE_ROM_CTRL__ROM) begin
      dev_sel_s1n_17 = 4'd0;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_ROM_CTRL__REGS)) == ADDR_SPACE_ROM_CTRL__REGS) begin
      dev_sel_s1n_17 = 4'd1;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_SRAM_CTRL_MAIN__RAM)) == ADDR_SPACE_SRAM_CTRL_MAIN__RAM) begin
      dev_sel_s1n_17 = 4'd2;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_FLASH_CTRL__MEM)) == ADDR_SPACE_FLASH_CTRL__MEM) begin
      dev_sel_s1n_17 = 4'd3;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_PERI)) == ADDR_SPACE_PERI) begin
      dev_sel_s1n_17 = 4'd4;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_FLASH_CTRL__CORE)) == ADDR_SPACE_FLASH_CTRL__CORE) begin
      dev_sel_s1n_17 = 4'd5;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_FLASH_CTRL__PRIM)) == ADDR_SPACE_FLASH_CTRL__PRIM) begin
      dev_sel_s1n_17 = 4'd6;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_AES)) == ADDR_SPACE_AES) begin
      dev_sel_s1n_17 = 4'd7;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_RV_PLIC)) == ADDR_SPACE_RV_PLIC) begin
      dev_sel_s1n_17 = 4'd8;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_SRAM_CTRL_MAIN__REGS)) == ADDR_SPACE_SRAM_CTRL_MAIN__REGS) begin
      dev_sel_s1n_17 = 4'd9;

    end else if ((tl_s1n_17_us_h2d.a_address &
                  ~(ADDR_MASK_RV_CORE_IBEX__CFG)) == ADDR_SPACE_RV_CORE_IBEX__CFG) begin
      dev_sel_s1n_17 = 4'd10;
end
  end


  // Instantiation phase
  tlul_socket_1n #(
    .HReqDepth (4'h0),
    .HRspDepth (4'h0),
    .DReqDepth (12'h0),
    .DRspDepth (12'h0),
    .N         (3)
  ) u_s1n_13 (
    .clk_i        (clk_main_i),
    .rst_ni       (rst_main_ni),
    .tl_h_i       (tl_s1n_13_us_h2d),
    .tl_h_o       (tl_s1n_13_us_d2h),
    .tl_d_o       (tl_s1n_13_ds_h2d),
    .tl_d_i       (tl_s1n_13_ds_d2h),
    .dev_select_i (dev_sel_s1n_13)
  );
  tlul_socket_m1 #(
    .HReqDepth (8'h0),
    .HRspDepth (8'h0),
    .DReqDepth (4'h0),
    .DRspDepth (4'h0),
    .M         (2)
  ) u_sm1_14 (
    .clk_i        (clk_main_i),
    .rst_ni       (rst_main_ni),
    .tl_h_i       (tl_sm1_14_us_h2d),
    .tl_h_o       (tl_sm1_14_us_d2h),
    .tl_d_o       (tl_sm1_14_ds_h2d),
    .tl_d_i       (tl_sm1_14_ds_d2h)
  );
  tlul_socket_m1 #(
    .HReqDepth (8'h0),
    .HRspDepth (8'h0),
    .DReqDepth (4'h0),
    .DRspDepth (4'h0),
    .M         (2)
  ) u_sm1_15 (
    .clk_i        (clk_main_i),
    .rst_ni       (rst_main_ni),
    .tl_h_i       (tl_sm1_15_us_h2d),
    .tl_h_o       (tl_sm1_15_us_d2h),
    .tl_d_o       (tl_sm1_15_ds_h2d),
    .tl_d_i       (tl_sm1_15_ds_d2h)
  );
  tlul_socket_m1 #(
    .HReqDepth (8'h0),
    .HRspDepth (8'h0),
    .DRspPass  (1'b0),
    .M         (2)
  ) u_sm1_16 (
    .clk_i        (clk_main_i),
    .rst_ni       (rst_main_ni),
    .tl_h_i       (tl_sm1_16_us_h2d),
    .tl_h_o       (tl_sm1_16_us_d2h),
    .tl_d_o       (tl_sm1_16_ds_h2d),
    .tl_d_i       (tl_sm1_16_ds_d2h)
  );
  tlul_socket_1n #(
    .HReqDepth (4'h0),
    .HRspDepth (4'h0),
    .DRspPass  (11'h79f),
    .DReqDepth (44'h1100000),
    .DRspDepth (44'h1100000),
    .N         (11)
  ) u_s1n_17 (
    .clk_i        (clk_main_i),
    .rst_ni       (rst_main_ni),
    .tl_h_i       (tl_s1n_17_us_h2d),
    .tl_h_o       (tl_s1n_17_us_d2h),
    .tl_d_o       (tl_s1n_17_ds_h2d),
    .tl_d_i       (tl_s1n_17_ds_d2h),
    .dev_select_i (dev_sel_s1n_17)
  );
  tlul_fifo_async #(
    .ReqDepth        (1),
    .RspDepth        (1)
  ) u_asf_18 (
    .clk_h_i      (clk_main_i),
    .rst_h_ni     (rst_main_ni),
    .clk_d_i      (clk_fixed_i),
    .rst_d_ni     (rst_fixed_ni),
    .tl_h_i       (tl_asf_18_us_h2d),
    .tl_h_o       (tl_asf_18_us_d2h),
    .tl_d_o       (tl_asf_18_ds_h2d),
    .tl_d_i       (tl_asf_18_ds_d2h)
  );

endmodule
