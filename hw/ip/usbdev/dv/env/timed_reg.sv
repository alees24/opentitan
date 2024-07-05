// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Description of a predicted change of a register field.
typedef struct {
  // Unique ID of predicted change; diagnostically useful in log messages.
  int unsigned    id;
  // Previous value of this field.
  uvm_reg_data_t  val_prev;
  // New value of this field.
  uvm_reg_data_t  val_new;
  // Latest time of predicted change.
  int             latest_time;
} timed_field_pred_t;

// Description of a register field for which timed predictions are to be made.
class timed_reg_field extends uvm_object;
  `uvm_object_utils(timed_reg_field)

  `uvm_object_new

  // Register field within DV/RAL.
  dv_base_reg_field  field;

  // Mask for this register field within the parent register.
  uvm_reg_data_t     mask;
  // Bit number of LSB of the register field within the parent register.
  int unsigned       lsb;

  // Maximum expected delay for a predicted change to be met.
  int unsigned       max_delay;

  // Latest matched value of this field; used in predicting CSR read data.
  uvm_reg_data_t     read_latest;

  // Latest prediction for this register field; used for predicting CSR read data.
  bit                pred_valid;
  timed_field_pred_t pred_latest;

  // Latest value of this field observed by the `timed_checker` process.
  uvm_reg_data_t     obs_latest;

  // Queue of predictions for this register field; used by the `timed_checker` process.
  timed_field_pred_t pred_q[$];
endclass

// Description of a register for which timed-limited predictions are to be made.
class timed_reg extends uvm_object;
  `uvm_object_utils(timed_reg)

  `uvm_object_new

  // Register bits that we cannot predict; this may be ascertained by consulting the bitmaps
  // of the individual register fields, but we keep a copy for convenience and fast access.
  uvm_reg_data_t unpred_mask = {`UVM_REG_DATA_WIDTH{1'b1}};

  // Predicted fields within this register.
  timed_reg_field fields[$];

  // Add a field to the description of this register.
  //
  // `init_val` specifies the expected initial value of the field (ie. post-DUT reset).
  // `max_delay` gives the maximum expected delay before a prediction is met (in DUT clock cycles).
  function void add_field(ref dv_base_reg_field field, input uvm_reg_data_t init_val,
                          int unsigned max_delay);
    timed_reg_field trf = timed_reg_field::type_id::create("trf");
    trf.field = field;
    // Retain properties of register field; convenience and efficiency
    // (available less directly within `dv_base_reg_field`).
    trf.lsb = field.get_lsb_pos();
    trf.mask = ((1 << field.get_n_bits()) - 1) << trf.lsb;
    // Remember the maximum prediction delay.
    trf.max_delay = max_delay;
    // Update the 'unpredictable bits' mask.
    unpred_mask &= ~trf.mask;
    // State for CSR reading.
    trf.read_latest = init_val;
    trf.pred_valid = 1'b0;
    // State for `timed_checker` process.
    trf.obs_latest = init_val;
    fields.push_back(trf);
  endfunction
endclass
