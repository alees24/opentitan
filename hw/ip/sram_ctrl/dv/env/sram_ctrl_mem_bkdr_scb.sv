// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

class sram_ctrl_mem_bkdr_scb extends mem_bkdr_scb;

  `uvm_object_utils(sram_ctrl_mem_bkdr_scb)
  `uvm_object_new

  sram_ctrl_bkdr_util sram_ctrl_bkdr_util_h;

  protected otp_ctrl_pkg::sram_key_t   key;
  protected otp_ctrl_pkg::sram_nonce_t nonce;

  virtual function mem_data_t get_bkdr_val(mem_addr_t addr);
    // This chops the integrity bits since mem_data_t is just the data portion.
    return sram_ctrl_bkdr_util_h.sram_encrypt_read32_integ(addr, key, nonce, 0);
  endfunction

  virtual function void update_key(otp_ctrl_pkg::sram_key_t   key,
                                   otp_ctrl_pkg::sram_nonce_t nonce);
    this.key   = key;
    this.nonce = nonce;
  endfunction

endclass
