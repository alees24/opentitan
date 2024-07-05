// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// `Timed registers` cannot be guaranteed to change at an exact time, so a certain amount of
// variability must be expected in the timing.
class usbdev_timed_regs extends uvm_object;
  `uvm_object_utils(usbdev_timed_regs)

  `uvm_object_new

  typedef enum {
    TimedIntrState = 0,
    TimedUsbStat,
    TimedInSent,
    TimedWakeEvents,
    // TimedConfigIn0-.. must have contiguous, ascending values.
    TimedConfigIn0,
    TimedConfigIn1,
    TimedConfigIn2,
    TimedConfigIn3,
    TimedConfigIn4,
    TimedConfigIn5,
    TimedConfigIn6,
    TimedConfigIn7,
    TimedConfigIn8,
    TimedConfigIn9,
    TimedConfigIn10,
    TimedConfigIn11
  } timed_reg_e;

  // Access to DUT clock.
  virtual clk_rst_if clk_rst_vif;
  // Access to DUT registers.
  usbdev_reg_block ral;

  timed_reg timed[timed_reg_e];

  // Prediction ID; diagnostic purposes only.
  int unsigned pred_id = 0;

  // Monotonic, wrapping cycle count; used to detect when expectations have not been met.
  int time_now = 0;

  // Perform a read of the actual DUT register state, for checking against expectations.
  // Note: we perform a backdoor read to avoid impacting the timing of the DUT and DV.
  task read_act_data(timed_reg_e r, output uvm_reg_data_t act_data);
    case (r)
      TimedIntrState:  csr_rd(.ptr(ral.intr_state),   .value(act_data), .backdoor(1));
      TimedUsbStat:    csr_rd(.ptr(ral.usbstat),      .value(act_data), .backdoor(1));
      TimedInSent:     csr_rd(.ptr(ral.in_sent[0]),   .value(act_data), .backdoor(1));
      TimedConfigIn0:  csr_rd(.ptr(ral.configin[0]),  .value(act_data), .backdoor(1));
      TimedConfigIn1:  csr_rd(.ptr(ral.configin[1]),  .value(act_data), .backdoor(1));
      TimedConfigIn2:  csr_rd(.ptr(ral.configin[2]),  .value(act_data), .backdoor(1));
      TimedConfigIn3:  csr_rd(.ptr(ral.configin[3]),  .value(act_data), .backdoor(1));
      TimedConfigIn4:  csr_rd(.ptr(ral.configin[4]),  .value(act_data), .backdoor(1));
      TimedConfigIn5:  csr_rd(.ptr(ral.configin[5]),  .value(act_data), .backdoor(1));
      TimedConfigIn6:  csr_rd(.ptr(ral.configin[6]),  .value(act_data), .backdoor(1));
      TimedConfigIn7:  csr_rd(.ptr(ral.configin[7]),  .value(act_data), .backdoor(1));
      TimedConfigIn8:  csr_rd(.ptr(ral.configin[8]),  .value(act_data), .backdoor(1));
      TimedConfigIn9:  csr_rd(.ptr(ral.configin[9]),  .value(act_data), .backdoor(1));
      TimedConfigIn10: csr_rd(.ptr(ral.configin[10]), .value(act_data), .backdoor(1));
      TimedConfigIn11: csr_rd(.ptr(ral.configin[11]), .value(act_data), .backdoor(1));
      TimedWakeEvents: csr_rd(.ptr(ral.wake_events),  .value(act_data), .backdoor(1));
      default: `uvm_fatal(`gfn, "Invalid/unrecognized register")
    endcase
    `uvm_info(`gfn, $sformatf("Backdoor read of reg %p yielded 0x%0x", r, act_data), UVM_MEDIUM)
  endtask

  // Add a timed, predicted state change to the list of expectations for the given register.
  function void predict(timed_reg_e r, uvm_reg_data_t prev_data, uvm_reg_data_t new_data);
    uvm_reg_data_t changed_mask = prev_data ^ new_data;

    `uvm_info(`gfn, $sformatf("Expecting reg %p <= 0x%0x, from 0x%0x (mask 0x%0x), time_now 0x%0x",
                              r, new_data, prev_data, changed_mask, time_now), UVM_MEDIUM)

    // Post predictions for the changed register fields.
    for (int unsigned f = 0; f < timed[r].fields.size(); f++) begin
      if (changed_mask & timed[r].fields[f].mask) begin
        // This field is expected to change; form a prediction.
        timed_field_pred_t pred;
        pred.id          = pred_id++;
        pred.val_prev    = (prev_data & timed[r].fields[f].mask) >> timed[r].fields[f].lsb;
        pred.val_new     = (new_data  & timed[r].fields[f].mask) >> timed[r].fields[f].lsb;
        pred.latest_time = time_now + int'(timed[r].fields[f].max_delay);
        `uvm_info(`gfn, $sformatf(" - field '%s' <= 0x%0x from 0x%0x by time 0x%0x",
                                  timed[r].fields[f].field.get_name(), pred.val_new, pred.val_prev,
                                  pred.latest_time), UVM_MEDIUM)
        // Present this expectation for validating CSR reads from the DUT.
        timed[r].fields[f].pred_latest = pred;
        timed[r].fields[f].pred_valid  = 1'b1;
        // Present it to the checker also.
        timed[r].fields[f].pred_q.push_back(pred);
      end
    end
  endfunction

  // Form a prediction of a register field to check TL-UL read data.
  function uvm_reg_data_t form_pred(timed_reg_e r, ref timed_reg_field field,
                                    uvm_reg_data_t act_data);
    timed_field_pred_t pred = field.pred_latest;
    // Register bit mask for this field.
    uvm_reg_data_t f_mask = field.mask;
    // Predicted value of this register field.
    uvm_reg_data_t pred_val;
    `uvm_info(`gfn, $sformatf(" - field %s : pred valid %d prev 0x%0x new 0x%0x",
                              field.field.get_name(), field.pred_valid, pred.val_prev,
                              pred.val_new), UVM_MEDIUM)
    if (field.pred_valid) begin
      uvm_reg_data_t act_val = (act_data & f_mask) >> field.lsb;
      `uvm_info(`gfn, $sformatf("   (time_now 0x%0x latest_time 0x%0x)", pred.latest_time,
                                time_now), UVM_MEDIUM)
      if (act_val == pred.val_new) begin
        `uvm_info(`gfn, "   Prediction met", UVM_MEDIUM)
        pred_val = pred.val_new;
        // Prediction met; no longer required.
        field.pred_valid = 1'b0;
      end else begin
        `uvm_info(`gfn, $sformatf("   Prediction not met; act 0x%0x", act_val), UVM_MEDIUM)
        `DV_CHECK_EQ(act_val, pred.val_prev)
        `DV_CHECK_GT(pred.latest_time - time_now, 0)
        pred_val = pred.val_prev;
      end
      // Retain the latest prediction.
      field.read_latest = pred_val;
    end else begin
      // We have no new prediction, use the most recent prediction.
      pred_val = field.read_latest;
    end
    `uvm_info(`gfn, $sformatf("   (predicted as 0x%0x)", pred_val), UVM_MEDIUM)
    return pred_val;
  endfunction

  // Check a DUT read from the specified register against any timed expectations.
  function uvm_reg_data_t read(timed_reg_e r, uvm_reg_data_t act_data);
    // Propagate the bits of the observed DUT register that we are presently unable to predict.
    uvm_reg_data_t pred_data = act_data & timed[r].unpred_mask;
    `uvm_info(`gfn, $sformatf("Producing prediction for %p, act_data 0x%0x", r, act_data),
              UVM_MEDIUM)

    // Collect predictions for all of the register fields that we can predict.
    for (int unsigned f = 0; f < timed[r].fields.size(); f++) begin
      uvm_reg_data_t pred_val = form_pred(r, timed[r].fields[f], act_data);
      // Collect prediction of this register field.
      pred_data |= pred_val << timed[r].fields[f].lsb;
    end
    return pred_data;
  endfunction

  // Check whether the given field of the specified register yet matches the predicted change;
  // object to expired predictions that have not been matched, and any observed value that is
  // neither the previous nor the expected new value of that field.
  function void check_pred(timed_reg_e r, ref timed_reg_field field, input uvm_reg_data_t act_val);
    do begin
      // Check whether the actual state of the DUT register matches the expectation.
      timed_field_pred_t pred = field.pred_q[0];
      if (act_val == pred.val_new) begin
        // This expectation has been met and may be discarded.
        `uvm_info(`gfn, $sformatf("Reg %p field %s (ID 0x%0x) met expectation 0x%0x", r,
                                  field.field.get_name(), pred.id, pred.val_new), UVM_MEDIUM)
        void'(field.pred_q.pop_front());
        field.obs_latest = act_val;
      end else if (!(pred.latest_time - time_now > 0)) begin
        `uvm_info(`gfn, $sformatf("Reg %p field %s (ID 0x%0x) did NOT meet expectation 0x%0x",
                                  r, field.field.get_name(), pred.id, pred.val_new), UVM_MEDIUM)
        `uvm_info(`gfn, $sformatf(" (time_now 0x%0x)", time_now), UVM_MEDIUM)
        // Report the mismatched value.
        `DV_CHECK_EQ(act_val, pred.val_new);
      end else begin
        // Check that it still matches the previous value.
        `DV_CHECK_EQ(act_val, pred.val_prev);
        break;
      end
    end while (field.pred_q.size() > 0);
  endfunction

  // This process checks every prediction that is made, using backdoor csr_rd in zero time to avoid
  // interfering with actual CSR reads and the timing of the simulation.
  task check_predictions();
    // Collect the initial values post-reset.
    wait (clk_rst_vif.rst_n === 1'b1);
    forever begin
      timed_reg_e r;
      // The checker cannot afford to wait a cycle for the prediction because the CSR change may
      /// occur within the current cycle.
      @(negedge clk_rst_vif.clk);
      time_now++;
      // Check each of the timed registers for expired expectations.
      r = r.first();
      forever begin
        uvm_reg_data_t act_data;
        bit got_act_data = 1'b0;
        for (int unsigned f = 0; f < timed[r].fields.size(); f++) begin
          // Check expectation queue first to avoid performing backdoor reads on every cycle!
          if (timed[r].fields[f].pred_q.size() > 0) begin
            // Something is expected to happen to this register field.
            uvm_reg_data_t act_val;
            bit act_changed;
            if (!got_act_data) begin
              read_act_data(r, act_data);
              got_act_data = 1'b1;
            end
            // Collect as many expired expectations as possible.
            act_val = (act_data & timed[r].fields[f].mask) >> timed[r].fields[f].lsb;
            act_changed = (act_val != timed[r].fields[f].obs_latest);
            if (act_changed) begin
              `uvm_info(`gfn, $sformatf("Checker observed reg %p field %p change 0x%0x -> 0x%0x",
                                        r, timed[r].fields[f].field.get_name(),
                                        timed[r].fields[f].obs_latest, act_val), UVM_MEDIUM)
            end
            check_pred(r, timed[r].fields[f], act_val);
          end
        end
        if (r == r.last()) break;
        r = r.next();
      end
    end
  endtask

endclass : usbdev_timed_regs
