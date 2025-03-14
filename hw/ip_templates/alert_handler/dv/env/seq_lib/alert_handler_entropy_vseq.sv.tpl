// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// this sequence enable entropy by writing 1 to the lock_regen register.

class ${module_instance_name}_entropy_vseq extends ${module_instance_name}_smoke_vseq;
  `uvm_object_utils(${module_instance_name}_entropy_vseq)

  `uvm_object_new

  // large number of num_trans to make sure covers all alerts and escalation pings
  constraint num_trans_c {
    num_trans inside {[400:1000]};
  }

  // increase the possibility to enable more alerts, because ${module_instance_name} only sends ping on
  // enabled alerts
  constraint enable_one_alert_c {
    alert_en        dist {'1 :/ 9, [0:('1-1'b1)] :/ 1};
    (~alert_regwen) dist {'1 :/ 9, [0:('1-1'b1)] :/ 1};
  }

  constraint sig_int_c {
    esc_int_err == 0;
  }

  constraint lock_bit_c {
    do_lock_config == 1;
  }

  constraint esc_accum_thresh_c {
    foreach (accum_thresh[i]) {accum_thresh[i] dist {[0:1] :/ 5, [2:10] :/ 5};}
  }

  function void pre_randomize();
    this.enable_classa_only_c.constraint_mode(0);
    verbosity = UVM_HIGH;
  endfunction

endclass : ${module_instance_name}_entropy_vseq
