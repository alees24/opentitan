// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// this sequence responses to escalation pins by sending the resp pins
class esc_receiver_esc_rsp_seq extends esc_receiver_base_seq;

  `uvm_object_utils(esc_receiver_esc_rsp_seq)

  extern constraint esc_receiver_esc_rsp_seq_c;

  extern function new (string name = "");

endclass : esc_receiver_esc_rsp_seq

constraint esc_receiver_esc_rsp_seq::esc_receiver_esc_rsp_seq_c {
  r_esc_rsp == 1;
}

function esc_receiver_esc_rsp_seq::new (string name = "");
  super.new(name);
endfunction : new
