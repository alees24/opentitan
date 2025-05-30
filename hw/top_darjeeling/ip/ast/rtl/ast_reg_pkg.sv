// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Package auto-generated by `reggen` containing data structure

package ast_reg_pkg;

  // Param list
  parameter int NumRegsB = 5;
  parameter int NumUsbBeaconPulses = 8;

  // Address widths within the block
  parameter int BlockAw = 10;

  // Number of registers for every interface
  parameter int NumRegs = 36;

  ////////////////////////////
  // Typedefs for registers //
  ////////////////////////////

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega0_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega1_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega2_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega3_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega4_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega5_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega6_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega7_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega8_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega9_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega10_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega11_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega12_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega13_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega14_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega15_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega16_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega17_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega18_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega19_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega20_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega21_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega22_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega23_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega24_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega25_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega26_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega27_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega28_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_rega29_reg_t;

  typedef struct packed {
    logic [31:0] q;
    logic        qe;
  } ast_reg2hw_regal_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } ast_reg2hw_regb_mreg_t;

  typedef struct packed {
    logic [31:0] d;
  } ast_hw2reg_regal_reg_t;

  // Register -> HW type
  typedef struct packed {
    ast_reg2hw_rega0_reg_t rega0; // [1152:1121]
    ast_reg2hw_rega1_reg_t rega1; // [1120:1089]
    ast_reg2hw_rega2_reg_t rega2; // [1088:1057]
    ast_reg2hw_rega3_reg_t rega3; // [1056:1025]
    ast_reg2hw_rega4_reg_t rega4; // [1024:993]
    ast_reg2hw_rega5_reg_t rega5; // [992:961]
    ast_reg2hw_rega6_reg_t rega6; // [960:929]
    ast_reg2hw_rega7_reg_t rega7; // [928:897]
    ast_reg2hw_rega8_reg_t rega8; // [896:865]
    ast_reg2hw_rega9_reg_t rega9; // [864:833]
    ast_reg2hw_rega10_reg_t rega10; // [832:801]
    ast_reg2hw_rega11_reg_t rega11; // [800:769]
    ast_reg2hw_rega12_reg_t rega12; // [768:737]
    ast_reg2hw_rega13_reg_t rega13; // [736:705]
    ast_reg2hw_rega14_reg_t rega14; // [704:673]
    ast_reg2hw_rega15_reg_t rega15; // [672:641]
    ast_reg2hw_rega16_reg_t rega16; // [640:609]
    ast_reg2hw_rega17_reg_t rega17; // [608:577]
    ast_reg2hw_rega18_reg_t rega18; // [576:545]
    ast_reg2hw_rega19_reg_t rega19; // [544:513]
    ast_reg2hw_rega20_reg_t rega20; // [512:481]
    ast_reg2hw_rega21_reg_t rega21; // [480:449]
    ast_reg2hw_rega22_reg_t rega22; // [448:417]
    ast_reg2hw_rega23_reg_t rega23; // [416:385]
    ast_reg2hw_rega24_reg_t rega24; // [384:353]
    ast_reg2hw_rega25_reg_t rega25; // [352:321]
    ast_reg2hw_rega26_reg_t rega26; // [320:289]
    ast_reg2hw_rega27_reg_t rega27; // [288:257]
    ast_reg2hw_rega28_reg_t rega28; // [256:225]
    ast_reg2hw_rega29_reg_t rega29; // [224:193]
    ast_reg2hw_regal_reg_t regal; // [192:160]
    ast_reg2hw_regb_mreg_t [4:0] regb; // [159:0]
  } ast_reg2hw_t;

  // HW -> register type
  typedef struct packed {
    ast_hw2reg_regal_reg_t regal; // [31:0]
  } ast_hw2reg_t;

  // Register offsets
  parameter logic [BlockAw-1:0] AST_REGA0_OFFSET = 10'h 0;
  parameter logic [BlockAw-1:0] AST_REGA1_OFFSET = 10'h 4;
  parameter logic [BlockAw-1:0] AST_REGA2_OFFSET = 10'h 8;
  parameter logic [BlockAw-1:0] AST_REGA3_OFFSET = 10'h c;
  parameter logic [BlockAw-1:0] AST_REGA4_OFFSET = 10'h 10;
  parameter logic [BlockAw-1:0] AST_REGA5_OFFSET = 10'h 14;
  parameter logic [BlockAw-1:0] AST_REGA6_OFFSET = 10'h 18;
  parameter logic [BlockAw-1:0] AST_REGA7_OFFSET = 10'h 1c;
  parameter logic [BlockAw-1:0] AST_REGA8_OFFSET = 10'h 20;
  parameter logic [BlockAw-1:0] AST_REGA9_OFFSET = 10'h 24;
  parameter logic [BlockAw-1:0] AST_REGA10_OFFSET = 10'h 28;
  parameter logic [BlockAw-1:0] AST_REGA11_OFFSET = 10'h 2c;
  parameter logic [BlockAw-1:0] AST_REGA12_OFFSET = 10'h 30;
  parameter logic [BlockAw-1:0] AST_REGA13_OFFSET = 10'h 34;
  parameter logic [BlockAw-1:0] AST_REGA14_OFFSET = 10'h 38;
  parameter logic [BlockAw-1:0] AST_REGA15_OFFSET = 10'h 3c;
  parameter logic [BlockAw-1:0] AST_REGA16_OFFSET = 10'h 40;
  parameter logic [BlockAw-1:0] AST_REGA17_OFFSET = 10'h 44;
  parameter logic [BlockAw-1:0] AST_REGA18_OFFSET = 10'h 48;
  parameter logic [BlockAw-1:0] AST_REGA19_OFFSET = 10'h 4c;
  parameter logic [BlockAw-1:0] AST_REGA20_OFFSET = 10'h 50;
  parameter logic [BlockAw-1:0] AST_REGA21_OFFSET = 10'h 54;
  parameter logic [BlockAw-1:0] AST_REGA22_OFFSET = 10'h 58;
  parameter logic [BlockAw-1:0] AST_REGA23_OFFSET = 10'h 5c;
  parameter logic [BlockAw-1:0] AST_REGA24_OFFSET = 10'h 60;
  parameter logic [BlockAw-1:0] AST_REGA25_OFFSET = 10'h 64;
  parameter logic [BlockAw-1:0] AST_REGA26_OFFSET = 10'h 68;
  parameter logic [BlockAw-1:0] AST_REGA27_OFFSET = 10'h 6c;
  parameter logic [BlockAw-1:0] AST_REGA28_OFFSET = 10'h 70;
  parameter logic [BlockAw-1:0] AST_REGA29_OFFSET = 10'h 74;
  parameter logic [BlockAw-1:0] AST_REGAL_OFFSET = 10'h 78;
  parameter logic [BlockAw-1:0] AST_REGB_0_OFFSET = 10'h 200;
  parameter logic [BlockAw-1:0] AST_REGB_1_OFFSET = 10'h 204;
  parameter logic [BlockAw-1:0] AST_REGB_2_OFFSET = 10'h 208;
  parameter logic [BlockAw-1:0] AST_REGB_3_OFFSET = 10'h 20c;
  parameter logic [BlockAw-1:0] AST_REGB_4_OFFSET = 10'h 210;

  // Reset values for hwext registers and their fields
  parameter logic [31:0] AST_REGAL_RESVAL = 32'h 1e;
  parameter logic [31:0] AST_REGAL_REG32_RESVAL = 32'h 1e;

  // Register index
  typedef enum int {
    AST_REGA0,
    AST_REGA1,
    AST_REGA2,
    AST_REGA3,
    AST_REGA4,
    AST_REGA5,
    AST_REGA6,
    AST_REGA7,
    AST_REGA8,
    AST_REGA9,
    AST_REGA10,
    AST_REGA11,
    AST_REGA12,
    AST_REGA13,
    AST_REGA14,
    AST_REGA15,
    AST_REGA16,
    AST_REGA17,
    AST_REGA18,
    AST_REGA19,
    AST_REGA20,
    AST_REGA21,
    AST_REGA22,
    AST_REGA23,
    AST_REGA24,
    AST_REGA25,
    AST_REGA26,
    AST_REGA27,
    AST_REGA28,
    AST_REGA29,
    AST_REGAL,
    AST_REGB_0,
    AST_REGB_1,
    AST_REGB_2,
    AST_REGB_3,
    AST_REGB_4
  } ast_id_e;

  // Register width information to check illegal writes
  parameter logic [3:0] AST_PERMIT [36] = '{
    4'b 1111, // index[ 0] AST_REGA0
    4'b 1111, // index[ 1] AST_REGA1
    4'b 1111, // index[ 2] AST_REGA2
    4'b 1111, // index[ 3] AST_REGA3
    4'b 1111, // index[ 4] AST_REGA4
    4'b 1111, // index[ 5] AST_REGA5
    4'b 1111, // index[ 6] AST_REGA6
    4'b 1111, // index[ 7] AST_REGA7
    4'b 1111, // index[ 8] AST_REGA8
    4'b 1111, // index[ 9] AST_REGA9
    4'b 1111, // index[10] AST_REGA10
    4'b 1111, // index[11] AST_REGA11
    4'b 1111, // index[12] AST_REGA12
    4'b 1111, // index[13] AST_REGA13
    4'b 1111, // index[14] AST_REGA14
    4'b 1111, // index[15] AST_REGA15
    4'b 1111, // index[16] AST_REGA16
    4'b 1111, // index[17] AST_REGA17
    4'b 1111, // index[18] AST_REGA18
    4'b 1111, // index[19] AST_REGA19
    4'b 1111, // index[20] AST_REGA20
    4'b 1111, // index[21] AST_REGA21
    4'b 1111, // index[22] AST_REGA22
    4'b 1111, // index[23] AST_REGA23
    4'b 1111, // index[24] AST_REGA24
    4'b 1111, // index[25] AST_REGA25
    4'b 1111, // index[26] AST_REGA26
    4'b 1111, // index[27] AST_REGA27
    4'b 1111, // index[28] AST_REGA28
    4'b 1111, // index[29] AST_REGA29
    4'b 1111, // index[30] AST_REGAL
    4'b 1111, // index[31] AST_REGB_0
    4'b 1111, // index[32] AST_REGB_1
    4'b 1111, // index[33] AST_REGB_2
    4'b 1111, // index[34] AST_REGB_3
    4'b 1111  // index[35] AST_REGB_4
  };

endpackage
