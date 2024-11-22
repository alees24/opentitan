// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module pwm_chan #(
  parameter int CntDw = 16,

  localparam int unsigned DcResnDw = $clog2(CntDw)
) (
  input                clk_i,
  input                rst_ni,

  input                pwm_en_i,
  input                invert_i,
  input                blink_en_i,
  input                htbt_en_i,
  input [CntDw-1:0]    phase_delay_i,
  input [CntDw-1:0]    duty_cycle_a_i,
  input [CntDw-1:0]    duty_cycle_b_i,
  input [CntDw-1:0]    blink_param_x_i,
  input [CntDw-1:0]    blink_param_y_i,

  input [CntDw-1:0]    phase_ctr_i,
  input [CntDw-1:0]    phase_ctr_next_i,
  input                cycle_end_i,
  input                clr_chan_cntr_i,
  input [DcResnDw-1:0] dc_resn_i,

  output logic         pwm_o
);

  logic [CntDw-1:0] phase_delay_masked, duty_cycle_masked;

  // Derive some additional control signals.
  logic blink_mode;
  logic htbt_mode;

  // Heartbeat mode is only available when blinking enabled, since some logic is shared.
  assign blink_mode = pwm_en_i & blink_en_i;
  assign htbt_mode = pwm_en_i & blink_en_i & htbt_en_i;

  // When there is a non-zero `phase_delay_i` we cannot use the `cycle_end_i` signal directly
  // to switch to a new duty cycle because we could be asserting the pwm output at that
  // instant, so we shall need to delay this signal.
  logic dc_update_pending;
  logic clr_dc_update;
  logic update;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dc_update_pending <= 1'b0;
    end else if (pwm_en_i) begin
      dc_update_pending <= (cycle_end_i & |phase_delay_i) & ~clr_dc_update;
    end
  end
  assign clr_dc_update = (phase_ctr_next_i >= phase_delay_masked);
  assign update = pwm_en_i & (|phase_delay_i ? (dc_update_pending & clr_dc_update) : cycle_end_i);

  logic [CntDw-1:0] duty_cycle_actual;
  logic [CntDw-1:0] on_phase;
  logic [CntDw-1:0] off_phase;
  logic             phase_wrap;
  logic             pwm_int;

  // Standard blink mode
  logic [CntDw-1:0] blink_ctr_q;
  logic [CntDw-1:0] blink_ctr_d;
  logic [CntDw-1:0] duty_cycle_blink;

  // Record whether we're in the 'b' phase of regular blinking; heartbeat mode
  // never leaves phase 'a'.
  logic blink_b_phase_q;
  logic blink_b_phase_d;

  // Final pulse cycle within this blinking phase?
  logic blink_end;
  assign blink_end = ~|blink_ctr_q;

  // Blink mode counts 'x'+1 pulse cycles and then, if heartbeat is disabled, 'y'+1 pulse cycles.
  // Non-heartbeat alternates between duty cycles 'a' and 'b' whereas heartbeat mode oscillates
  // linearly between the extremes of 'a' and 'b'.
  always_comb begin
    blink_ctr_d = blink_ctr_q;
    blink_b_phase_d = blink_b_phase_q;

    if (clr_chan_cntr_i) begin
      blink_ctr_d = blink_param_x_i;
      blink_b_phase_d = 1'b0;
    end else if (blink_mode & update) begin
      if (blink_end) begin
        blink_ctr_d = blink_b_phase_q ? blink_param_x_i : blink_param_y_i;
        blink_b_phase_d = !blink_b_phase_q & !htbt_en_i;
      end else begin
        blink_ctr_d = blink_ctr_q - 1;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      blink_ctr_q <= '0;
      blink_b_phase_q <= 1'b0;
    end else begin
      blink_ctr_q <= blink_ctr_d;
      blink_b_phase_q <= blink_b_phase_d;
    end
  end

  assign duty_cycle_blink = (blink_en_i && blink_b_phase_q) ? duty_cycle_b_i : duty_cycle_a_i;

  // Heartbeat mode
  logic [CntDw-1:0] duty_cycle_htbt;
  logic [CntDw-1:0] dc_htbt_d;
  logic [CntDw-1:0] dc_htbt_q;
  logic dc_htbt_end;
  logic dc_htbt_end_q;

  assign dc_htbt_end = htbt_mode & update & blink_end;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dc_htbt_end_q <= '0;
    end else if (clr_chan_cntr_i) begin
      dc_htbt_end_q <= '0;
    end else begin
      dc_htbt_end_q <= dc_htbt_end;
    end
  end

  logic pattern_repeat;
  logic htbt_falling;
  logic dc_wrap;
  logic dc_wrap_q;
  logic pos_htbt;
  logic neg_htbt;
  assign pos_htbt = (duty_cycle_a_i < duty_cycle_b_i);
  assign pattern_repeat = (duty_cycle_a_i == duty_cycle_b_i);
  assign neg_htbt = !pos_htbt & !pattern_repeat;

  // We support either duty cycle or 'a' or 'b' being the greater.
  logic [CntDw-1:0] dc_htbt_max;
  logic [CntDw-1:0] dc_htbt_min;
  assign dc_htbt_max = pos_htbt ? duty_cycle_b_i : duty_cycle_a_i;
  assign dc_htbt_min = pos_htbt ? duty_cycle_a_i : duty_cycle_b_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      htbt_falling <= '0;
    end else if (clr_chan_cntr_i) begin
      // For proper initialization, set the initial htbt_direction whenever a register is updated,
      // as indicated by clr_chan_cntr_i
      htbt_falling <= !pos_htbt;
    end else if (htbt_mode) begin
      if (!htbt_falling && ((dc_htbt_q >= dc_htbt_max) || (dc_wrap && dc_htbt_end))) begin
        htbt_falling <= 1'b1; // duty cycle counts down
      end else if (htbt_falling && ((dc_htbt_q <= dc_htbt_min) || (dc_wrap && dc_htbt_end))) begin
        htbt_falling <= 1'b0; // duty cycle counts up
      end
    end
  end

  localparam int CntExtDw = CntDw + 1;
  always_comb begin
    dc_wrap   = 1'b0;
    dc_htbt_d = dc_htbt_q;

    if (pattern_repeat) begin
      dc_htbt_d = duty_cycle_a_i;
    end else if (htbt_falling) begin
      {dc_wrap, dc_htbt_d} = (CntExtDw)'(dc_htbt_q) - (CntExtDw)'(blink_param_y_i) - 1'b1;
    end else begin
      {dc_wrap, dc_htbt_d} = (CntExtDw)'(dc_htbt_q) + (CntExtDw)'(blink_param_y_i) + 1'b1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dc_wrap_q <= 1'b0;
    end else if (clr_chan_cntr_i) begin
      dc_wrap_q <= 1'b0;
    end else if (dc_htbt_end) begin
      // To pick the first dc_wrap pulse during the transition and set the correct duration of
      // dc_wrap_q, pos_htbt and htbt_falling signals are involved
      dc_wrap_q <= pos_htbt ? dc_wrap & ~htbt_falling : dc_wrap & htbt_falling;
    end
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dc_htbt_q <= '0;
    end else if (clr_chan_cntr_i) begin
      dc_htbt_q <= duty_cycle_a_i;
    end else if (dc_htbt_end) begin
      // Note that although the duty cycle still advances in the event of wrapping, because of the
      // nature of two's-complement arithmetic this is non-destructive, and the next update will
      // have the opposite sign and thus reinstate the previous value.
      dc_htbt_q <= dc_htbt_d;
    end
  end

  assign duty_cycle_htbt = dc_wrap_q ? {CntDw{pos_htbt}} : dc_htbt_q;

  assign duty_cycle_actual = htbt_mode ? duty_cycle_htbt : duty_cycle_blink;

  // For cases when the desired duty_cycle does not line up with the chosen resolution
  // we mask away any used bits.
  logic [CntDw-1:0] dc_mask;
  // Mask is de-asserted for first dc_resn_i + 1 bits.
  // e.g. for dc_resn = 12, dc_mask = 16'b0000_0000_0000_0111
  // Bits marked as one in this mask are unused in computing
  // turn-on or turn-off times
  assign dc_mask = {1'b0,{(CntDw-1){1'b1}}} >> dc_resn_i;

  // Explicitly round down the phase_delay and duty_cycle
  assign phase_delay_masked = phase_delay_i & ~dc_mask;
  assign duty_cycle_masked  = duty_cycle_actual & ~dc_mask;

  assign on_phase                = phase_delay_masked;
  assign {phase_wrap, off_phase} = {1'b0, phase_delay_masked} +
                                   {1'b0, duty_cycle_masked};

  logic on_phase_started;
  logic off_phase_started;

  assign on_phase_started  = (phase_ctr_i >= on_phase);
  assign off_phase_started = (phase_ctr_i >= off_phase);

  assign pwm_int = !pwm_en_i ? 1'b0 :
                   phase_wrap ? on_phase_started | ~off_phase_started :
                                on_phase_started & ~off_phase_started;

  assign pwm_o = invert_i ? ~pwm_int : pwm_int;

endmodule : pwm_chan
