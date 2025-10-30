// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C Target transceiver.
//
// - Target (including Secondary Controller) functionality.
// - This logic is reactive, responding to the SCL clock signal that it receives from the active
//   controller, and the word-level decoding is driven entirely by that clock before moving data
//   words and other signals across a CDC boundary into the main IP block.
// - Any command requiring an immediate response must also be implemented here, but data is
//   presented to this module preemptively by Target-side logic running within the main IP block
//   on its clock.

module i3c_target_trx
  import i3c_pkg::*;
#(
  // Number of target(s) or target group(s) presented simultaneously on the I3C bus, including the
  // Secondary Controller Role.
  parameter int unsigned NumTargets = 2,
  // Number of SDA lanes; must be one presently since HDR-BT mode not supported.
  parameter int unsigned NumSDALanes = 1
) (
  // No free-running clock from the IP block core; driven by controller-supplied SCL.
  // Asynchronous reset.
  input                           rst_ni,

  input                           sw_reset_i,

  // Target device descriptions.
  input  i3c_targ_info_t          targ_info_i[NumTargets],

  // Prefetch request to Target logic.
  output logic                    trx_ptoggle_o,
  output i3c_targ_trx_pre_t       trx_pre_o,

  // Request/state information from Target logic.
  input                           trx_dvalid_i[NumTargets],
  output logic                    trx_dready_o[NumTargets],
  input  i3c_targ_trx_req_t       trx_dreq_i[NumTargets],

  // Response to Target logic.
  output logic                    trx_rtoggle_o,
  output i3c_targ_trx_rsp_t       trx_rsp_o,

  // Secondary Controller address and validity indicator.
  input                     [6:0] ctrl_addr_i,
  input                           ctrl_addr_valid_i,

  // HDR pattern detection.
  input                           hdr_exit_det_i,
  input                           hdr_restart_det_i,

  // I3C I/O signaling.
  input                           scl_i,
  input                           scl_ni,
  input                           sda0_clk_i,
  input                           sda0_clk_ni,
  input         [NumSDALanes-1:0] sda_i,
  output        [NumSDALanes-1:0] sda_o,
  output logic                    sda_pp_en_o,
  output logic                    sda_od_en_o,

  // DFT-related controls.
  input                           scanmode_i
);

  import i3c_consts_pkg::*;

  // Bit index must count down from 18 for HDR-DDR words.
  localparam int unsigned BitW = 5;

  // We must also respond to the I3C Broadcast Address (7'h7e) as well as the configured targets,
  // and we use an additional encoding to denote 'no match.'
  localparam int unsigned TargIDW = $clog2(NumTargets + 2);
  localparam bit [TargIDW-1:0] TargID_Broadcast = TargIDW'(NumTargets);
  localparam bit [TargIDW-1:0] TargID_NoMatch   = TargIDW'(NumTargets+1);

  // Use token scheme to emit Double Data Rate signaling, rather than driving the MUX with the SCL
  // line directly.
  localparam bit UseTokens = 1'b1;

  // TODO: The sw reset functionality is going to need some consideration here, because this is
  // a synchronouse reset presently and logic driven by SDA/SCL cannot be reset synchronously.
  // We shall need to turn the sw reset function into a prolonged, multi-cycle asynchronous
  // assertion.

  // Ensure that we do not respond to our own SDA signaling.
  wire sda_en = sda_pp_en_o | sda_od_en_o;
  logic start_det, stop_det;

  // Capture HDR exit detection on rising SCL that indicate that start of the ensuing STOP.
  logic ddr_mode;
  logic hdr_exit;
  always_ff @(posedge scl_i or negedge rst_ni) begin
    if (!rst_ni) begin
      hdr_exit  <= 1'b0;
    end else begin
      hdr_exit  <= hdr_exit_det_i;
    end
  end

  // SDR START detection.
  always_ff @(posedge sda0_clk_ni or negedge rst_ni) begin
    if (!rst_ni) begin
      start_det <= 1'b0;
    end else if (!sda_en & !ddr_mode) begin
      start_det <= scl_i;
    end
  end
  // SDR STOP detection.
  always_ff @(posedge sda0_clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      stop_det  <= 1'b0;
    end else if (start_det) begin
      stop_det  <= 1'b0;
    end else if (!sda_en & (!ddr_mode | hdr_exit)) begin
      stop_det  <= scl_i;
    end
  end

  // Target ID; this is either the unique ID number of a configured Target, or TargID_Broadcast.
  logic [TargIDW-1:0] targ_id_q;

  // Transmission of Read Data to the Controller.
  // TODO: We can probably drop the LSBs, right?
  logic [9:0] tx_data_nq[NumSDALanes];
  logic [9:0] tx_data_pq[NumSDALanes];
  // For HDR-DDR, odd-indexed bits are SCL-negedge clocked, i.e. including the MSB.
  // For SDR, only these bits are used.
  assign tx_data_nq[0] = trx_dreq_i[targ_id_q].rdata_nq;
  // For HDR-DDR, even-indexed bits are SCL-posedge clocked, i.e. including the LSB.
  assign tx_data_pq[0] = trx_dreq_i[targ_id_q].rdata_pq;

  // Parity and CRC-5 are calculated on all transmitted and received Command and Data Words in
  // HDR-DDR mode. When transmitting, the calculated values are inserted as the data is sent.
  logic [1:0] parity_q;
  logic [1:0] parity_d;
  logic [4:0] crc5_q;
  logic [4:0] crc5_d;
  logic parity_nq_emit;
  logic parity_pq_emit;
  logic crc5_nq_emit;
  logic crc5_pq_emit;

  // Positive-edge data.
  // - we have 10 data bits for each clock edge to support the collection of a 20-bit HDR-DDR word.
  logic [9:0] sda_pq[NumSDALanes];
  logic sda_pq_shift;
  logic sda_pq_load;
  always_ff @(posedge scl_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sda_pq  <= '{'b0};
    end else if (crc5_pq_emit) begin : crc5_pq
      // Load the even-indexed bits of the CRC Word from the calculcated CRC-5 value.
      sda_pq[0][9:7] <= {crc5_q[3], crc5_q[1], 1'b1};  // Park SDA high for bus turnaround.
    end else if (parity_pq_emit) begin : parity_pq
      sda_pq[0][9] <= parity_d[0];
    end else if (sda_pq_load) begin : load_pq
      for (int unsigned lane = 0; lane < NumSDALanes; lane++) begin
        sda_pq[lane]  <= tx_data_pq[lane];
      end
    end else if (sda_pq_shift) begin : shift_pq
      for (int unsigned lane = 0; lane < NumSDALanes; lane++) begin
        sda_pq[lane]  <= {sda_pq[lane][8:0], sda_i[lane]};
      end
   end
  end

  // Negative-edge data (DDR signaling).
  logic [9:0] sda_nq[NumSDALanes];
  logic sda_nq_shift;
  logic sda_nq_load;
  always_ff @(posedge scl_ni or negedge rst_ni) begin
    if (!rst_ni) begin
      sda_nq  <= '{'b0};
    end else if (crc5_nq_emit) begin : crc5_nq
      // Load the odd-indexed bits of the CRC Word from the calculated CRC-5 value.
      sda_nq[0][9:7] <= {crc5_q[4], crc5_q[2], crc5_q[0]};
    end else if (parity_nq_emit) begin : parity_nq
      sda_nq[0][9] <= parity_d[1];
    end else if (sda_nq_load) begin : load_nq
      for (int unsigned lane = 0; lane < NumSDALanes; lane++) begin
        sda_nq[lane]  <= tx_data_nq[lane];
      end     
    end else if (sda_nq_shift) begin : shift_nq
      for (int unsigned lane = 0; lane < NumSDALanes; lane++) begin
        sda_nq[lane]  <= {sda_nq[lane][8:0], sda_i[lane]};
      end
    end
  end

  // Properties of current transfer. TODO: terminology?
  // logic [7:0] xfer_cmd;
  // TODO:

  // Check whether the arbitrated address matches against our configuration.
  wire [6:0] arb_addr = sda_pq[0][6:0];
  logic [TargIDW-1:0] targ_id;
  always_comb begin
    targ_id = TargID_NoMatch;
    // Respond to the I3C Broadcast Address.
    if (arb_addr == Addr_Broadcast) targ_id = TargID_Broadcast;
    else begin
      // Secondary Controller overrides the configuration for the first target; test the configured
      // targets in descending order such that the first target has the highest priority.
      for (int t = NumTargets - 1; t >= 0; t--) begin
        // TODO: Consider what should happen if the configuration is valid but the Controller is
        // not in Secondary Controller mode.
        bit [6:0] mask, match;
        mask  = targ_info_i[t].mask | {7{!t && ctrl_addr_valid_i}};
        match = (t || !ctrl_addr_valid_i) ? targ_info_i[t].addr : ctrl_addr_i;
        // A mask value of 0 invalidates that configuration entry, for compatibility with the
        // OpenTitan I2C target logic.
        if ((arb_addr & mask) == match && |mask) targ_id = t;
      end
    end
  end

  typedef enum logic [4:0] {
    State_Idle,
    State_PreStop,

    // --- Address arbitration ---
    State_ArbReq,
    State_ArbLost,
    State_AckAddr,

    // TODO: ACK/NAK of command (first data word).
    // TODO: Must respond to Controller abort of read Data Word.

    // --- Transmission ---
    State_TxSDR,
    State_TxDataDDR,
    State_TxCRCDDR,
    State_TxNACKDDR,

    // --- Reception ---
    State_RxSDR,
    State_RxCmdDDR,
    State_RxPreDDR,
    State_RxDataDDR,
    State_RxCRCDDR,
    State_RxRsvdDDR,

    // --- Ignoring traffic, e.g. HDR mode that is not understood. ---
    State_Ignore
  } state_e;

  // Transceiver state.
  state_e state_q, state_d;
  logic [BitW-1:0] bit_idx;
  // Indications of pre-penultimate, penultimate and the last bit; these are useful for both
  // the addressing phase and the reception of HDR-DDR words.
  wire prepen_bit = (bit_idx == BitW'('d2));
  wire penult_bit = (bit_idx == BitW'('d1));
  wire last_bit = ~|bit_idx;

  // Receiving data on both clock edges?
  wire rx_ddr = state_q inside {State_RxCmdDDR, State_RxPreDDR, State_RxDataDDR, State_RxCRCDDR};
  // Transmitting data on both clock edges?
  wire tx_ddr = state_q inside {State_TxDataDDR, State_TxCRCDDR};

  // HDR Exit detection must be responsive at all times.
  always_ff @(posedge scl_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ddr_mode  <= 1'b0;
    end else begin
      ddr_mode <= (rx_ddr || tx_ddr) & ~hdr_exit_det_i;
    end
  end

  // TODO:
  wire arb_reqd = 1'b0;
  // Arbitration is lost if we're requesting a '1' but the line was pulled to '0'.
  wire arb_lost = &{(state_q == State_ArbReq), sda_o, !sda_i[0]};

// TODO: We have no concept of (SDR) Sr presently.

  // TODO:
  wire starting = (state_q == State_Idle || hdr_exit) & start_det;
// TODO: Neither the above nor this below is correct...need to address start detection and
// interaction with scl-negedge-triggered state_q!
//wire starting = start_det;

  // Is there something to transmit on behalf of the address target?
  wire tx_avail = (targ_id_q < NumTargets) && trx_dvalid_i[targ_id_q];
  // Is it a request to send the CRC word at the end of an HDR-DDR read?
  wire tx_send_crc = !trx_dreq_i[targ_id_q].rdata_nq[9];

  // Are we being asked for read data that we have available?
  wire tx_avail_sdr = &{tx_avail, last_bit, !arb_lost};
  wire tx_avail_ddr = &{tx_avail, last_bit, cmd_word.rnw};
  // TODO:
  wire tx_repeat    = &{tx_avail, last_bit, !tx_send_crc};
wire tx_done_ddr = tx_avail && last_bit && tx_send_crc;
  // Starting transmission of a new data unit.
  wire tx_starting  = |{(state_q == State_ArbReq)    & tx_avail_sdr,
                        (state_q == State_RxCmdDDR)  & tx_avail_ddr,
                        (state_q == State_TxSDR)     & tx_repeat,
                        (state_q == State_TxDataDDR) & tx_repeat,
                        (state_q == State_TxDataDDR) & tx_done_ddr};
  // Ending transmission of the current data unit (Note: may also be starting another).
  wire tx_ending     = last_bit & (state_q inside {State_TxSDR, State_TxCRCDDR});
  assign sda_nq_load = tx_starting |
  // TODO:
    (state_q == State_TxDataDDR && tx_done_ddr);
  // SCL-positive shift register must be loaded a half-cycle later.
  always_ff @(posedge scl_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sda_pq_load     <= 1'b0;
      sda_pq_shift    <= 1'b0;
      crc5_pq_emit    <= 1'b0;
      parity_pq_emit  <= 1'b0;
    end else begin
      sda_pq_load     <= sda_nq_load;
      sda_pq_shift    <= sda_nq_shift;
      crc5_pq_emit    <= crc5_nq_emit;
      parity_pq_emit  <= crc5_nq_emit;
    end
  end
  // TODO: Can we sensibly clock-gate this?
  assign sda_nq_shift = 1'b1;

  wire [7:0] rx_ccc = sda_pq[0][8:1];
  wire enthdr_det = ((rx_ccc >> 3) == (ENTHDR0 >> 3)) & last_bit;  // There are 8 HDR modes.
  wire entddr_det = (rx_ccc == ENTHDR0) & last_bit;

  // Preamble bits at the start of HDR-DDR word.
  wire [1:0] ddr_pre = {sda_pq[0][0], sda_i[0]};

  wire addr_matched = (targ_id_q != TargID_NoMatch);
  wire send_ack = &{state_q == State_ArbLost, penult_bit, addr_matched};

  // Combine the SCL-posedge and SCL-negedge shift registers to present the received HDR-DDR word.
  i3c_ddr_cmd_word_t cmd_word;
  i3c_targ_trx_rsp_t rsp;
  logic [15:0] ddr_payload;
  always_comb begin
    // Not yet captured PAR0, but we have already shifted in PAR1, so the loop collects
    ddr_payload[15] = sda_pq[0][7];
    for (int unsigned b = 0; b < 7; b++) begin
      ddr_payload[2*b+2] = sda_nq[0][b];
      ddr_payload[2*b+1] = sda_pq[0][b];
    end
    // This is the LSB of the data payload, which has not yet been captured.
    ddr_payload[0] = sda_i[0];
  end
  // Reinterpret the HDR-DDR word.
  assign cmd_word = i3c_ddr_cmd_word_t'(rsp.wdata[15:0]);

  always_comb begin
    state_d = state_q;
    // HDR Exit detection must be responsive at all times.
//    if (hdr_exit) begin
//      // HDR Exit is expected to be followed by a STOP, but that could be the beginning of 
//      // SDR Restart signaling.
//      state_d = State_RxSDR;
//    end else
    if (starting) begin
      // START handling.
      state_d = arb_reqd ? State_ArbReq : State_ArbLost;
    end else if (stop_det) begin
      // STOP handling.
      state_d = State_Idle;
    end else begin
      case (state_q)
        // --- Address Arbitration ---
        // TODO: Incomplete.
        State_ArbReq:   state_d = last_bit ? (arb_lost ? State_RxSDR   : State_TxSDR)
                                           : (arb_lost ? State_ArbLost : State_ArbReq);
        State_ArbLost:  state_d = last_bit ? State_RxSDR :
                              ((penult_bit & addr_matched) ? State_AckAddr : State_ArbLost);
        State_AckAddr:  state_d = State_RxSDR;
        // --- Word Reception; not necessarily addressed, but another device is transmitting. ---
        State_RxSDR: begin
          // We must detect entry to any HDR mode, though we understand only HDR-DDR.
          state_d = entddr_det ? State_RxCmdDDR : (enthdr_det ? State_Ignore : State_RxSDR);
        end
        // --- Expecting an HDR-DDR Command word ---
        // TODO: Check the expected 2'b01 preamble; what do we do if we get something else?
        State_RxCmdDDR: begin
          // Respond to Read/Write Command, considering whether or not there is data available.
          // - NACKing of Read Commands is always permitted.
          // - NACKing of Write Commands may be enabled, but is not permitted by default.
          state_d = last_bit ? (cmd_word.rnw ? (tx_avail ? State_TxDataDDR : State_TxNACKDDR)
                                             : State_RxPreDDR)
                             : State_RxCmdDDR;
        end
        // --- Expecting an HDR-DDR Data Word or CRC word ---
        State_RxPreDDR: begin
          case (ddr_pre)
            2'b00:   state_d = State_RxRsvdDDR;
            2'b01:   state_d = State_RxCRCDDR;
            default: state_d = State_RxDataDDR;
          endcase
        end
        State_RxDataDDR: state_d = last_bit ? State_RxPreDDR : State_RxDataDDR;
        State_RxCRCDDR:  state_d = last_bit ? State_RxCmdDDR : State_RxCRCDDR;
        State_RxRsvdDDR: state_d = last_bit ? State_RxPreDDR : State_RxRsvdDDR;
        // --- Word Transmission ---
        State_TxSDR:     state_d = (!last_bit | tx_repeat) ? State_TxSDR : State_Ignore;
        State_TxDataDDR: state_d = (!last_bit | tx_repeat) ? State_TxDataDDR : State_TxCRCDDR;
        State_TxCRCDDR:  state_d = last_bit ? State_RxCmdDDR : State_TxCRCDDR;
        // --- Ignoring traffic that is not understood; other HDR modes ---
        // TODO: Respond to `hdr_exit_det_i`, of course.
        State_Ignore:    state_d = State_Ignore;
        // --- Recovery from invalid states ---
        default:         state_d = State_Idle;
      endcase
    end
  end

  // Target state machine.
  always_ff @(posedge scl_ni or negedge rst_ni) begin
    if (!rst_ni) begin
      targ_id_q <= TargID_NoMatch;
      state_q   <= State_Idle;
    end else if (sw_reset_i) begin
      targ_id_q <= TargID_NoMatch;
      state_q   <= State_Idle;
    end else begin
      if (prepen_bit) begin
        case (state_q)
          State_ArbLost:  targ_id_q <= targ_id;  // TODO: Check!
          State_RxCmdDDR: targ_id_q <= cmd_word.targ_addr;
        endcase
      end
      state_q <= state_d;
    end
  end

  // Supply new data on SDA?
  wire transmitting = state_q inside {State_TxSDR, State_TxDataDDR, State_TxCRCDDR};
  wire tx_supply = transmitting;
  // Sample the current state of SDA?
  wire rx_sample = rx_ddr || (state_q inside {State_ArbReq, State_ArbLost, State_RxSDR});
  wire buf_shift = tx_supply | rx_sample;

  // Bit pair used to update the parity and CRC-5 calculations.
  wire [1:0] parcrc_bit = transmitting ? {sda_nq[0][9], sda_pq[0][9]} : {sda_pq[0][0], sda_i[0]};

  // Parity calculated on received/transmitted data.
  logic       parity_sdr;
  logic       upd_parity;
  logic       init_parity;
  assign parity_d = {(parity_q[1] & ~init_parity) ^ parcrc_bit[1],
                     (parity_q[0] |  init_parity) ^ parcrc_bit[0]};
  // For SDR transmission it suffices to consult only the first parity bit.
  assign parity_sdr = parity_q[0];
  // Parity has no history from one data unit to the next, so we just need to initialize it.
  assign init_parity = (bit_idx >= (ddr_mode ? BitW'(16) : BitW'(8)));
  assign upd_parity  = 1'b1;  // TODO: Drop this if this proves sufficient.

  // CRC-5 calculated on received/transmitted data.
  logic       init_crc;
  assign crc5_d = init_crc ? '1 : {crc5_q[2],
                                   crc5_q[1] ^ parcrc_bit[1] ^ crc5_q[4],
                                   crc5_q[0] ^ parcrc_bit[0] ^ crc5_q[3],
                                   crc5_q[4] ^ parcrc_bit[1],
                                   crc5_q[3] ^ parcrc_bit[0]};
  assign init_crc = (state_q == State_RxCmdDDR && bit_idx > 'h10);

// TODO: For HDR-DDR we know that it's counting down in pairs, so can at least ignore LSB and
// we should perhaps consider dropping bit 0 altogether.
  wire data_bit = (bit_idx >= 'h2 && bit_idx <= 'h10);
  assign upd_crc = (state_q inside {State_RxCmdDDR, State_RxDataDDR, State_TxDataDDR}) & data_bit;
//  assign upd_crc  = ((state_q == State_RxDataDDR) ||
//                     (state_q == State_RxCmdDDR  && bit_idx <= 'h10) ||
//                     (state_q == State_TxDataDDR && bit_idx <= 'h10)) && (bit_idx >= 'h2);

  // Do the calculated parity and CRC-5 values match against the received values?
  // TODO: Can we defer the parity checking slightly, to avoid the combinational signal
  // `parity_error` briefly becoming asserted and causing confusion?
  wire parity_match = (parcrc_bit == parity_q);
  wire parity_check = (state_q == State_RxCmdDDR || state_q == State_RxDataDDR) && last_bit;
  wire parity_error = parity_check & !parity_match;

  wire crc5_match = ({sda_nq[0][2], sda_pq[0][1], sda_nq[0][1],
                      sda_pq[0][0], sda_nq[0][0]} == crc5_q);
  wire crc5_check = (state_q == State_RxCRCDDR && last_bit);
  wire crc5_error = crc5_check & !crc5_match;

  // Transmission of parity and CRC_5.
  assign parity_nq_emit = (state_q == State_TxDataDDR) & prepen_bit;
  assign crc5_nq_emit = (state_q == State_TxCRCDDR) & (bit_idx == BitW'(6));

  // Bit counting within data unit, and calculation of parity/CRC-5 on the data bits.
  logic rx_sample_q;
  always_ff @(posedge scl_ni or negedge rst_ni) begin
    if (!rst_ni) begin
      crc5_q        <= '1;
      parity_q      <= 2'b01;
      bit_idx       <= '1;
      rx_sample_q   <= 1'b0;
    end else if (sw_reset_i) begin
      crc5_q        <= '1;
      parity_q      <= 2'b01;
      bit_idx       <= '1;
      rx_sample_q   <= 1'b0;
    end else begin
      if (starting) begin
        bit_idx <= BitW'(8);
      end else if (last_bit) begin
        case (state_q)
          // SDR traffic shall be followed by further SDR traffic or, if entering HDR-DDR mode,
          // a Command Word.
          State_RxSDR:     bit_idx <= entddr_det ? BitW'(18) : BitW'(8);
          // DDR signaling counts down two bits a time and should be zero for the final pair of
          // bits, which will be collected on the SCL negative edge, so we initialize to 'bits'-2.
          State_RxCmdDDR:  bit_idx <= (cmd_word.rnw & tx_avail) ? BitW'(18) : '0;
          State_RxDataDDR,
          State_RxCRCDDR,
          State_RxRsvdDDR: bit_idx <= '0;  // Preamble next.
          State_RxPreDDR:  bit_idx <= (ddr_pre == 2'b01) ? BitW'(8) : BitW'(16);
          // TODO: Perhaps we want a TxPreDDR state here, to handle the explicit request to send CRC?
          State_TxDataDDR: bit_idx <= tx_repeat ? BitW'(18) : BitW'(10);
          //State_ArbReq,
          //State_ArbLost,
          //State_AckAddr: bit_idx <= BitW'(8);
          default:         bit_idx <= BitW'(8);
        endcase
      end else begin
        bit_idx   <= bit_idx - BitW'(ddr_mode ? {buf_shift, 1'b0} : {1'b0, buf_shift});
      end

      // Conditionally update CRC-5 and parity.
      if (init_crc | upd_crc) crc5_q <= crc5_d;
      if (init_parity | upd_parity) parity_q <= parity_d;

      rx_sample_q   <= rx_sample & ~|bit_idx;
    end
  end

  // Output driver enables.
  always_ff @(posedge scl_ni or negedge rst_ni) begin
    if (!rst_ni) begin
      sda_pp_en_o <= 1'b0;
      sda_od_en_o <= 1'b0;
    end else if (sw_reset_i) begin
      // Disable drivers.
      sda_pp_en_o <= 1'b0;
      sda_od_en_o <= 1'b0;
    end else if (send_ack) begin
      // Acknowledge address.
      sda_pp_en_o   <= 1'b0;
      sda_od_en_o   <= 1'b1;
    end else if (tx_starting | tx_ending) begin
      sda_pp_en_o   <= tx_starting;
      sda_od_en_o   <= 1'b0;
    end
  end

  if (UseTokens) begin : use_tokens
    // We use a single-bit toggle that counts the even and odd clock _cycles_, and by propagating
    // the current toggle state through the rising edge of SCL we can know which of the pair of
    // data bits should be driving the SDA line in HDR-DDR transmission.
    //
    // Propagate the toggle bit to the SCL-rising logic.
    logic sda_out_ptog;
    always_ff @(posedge scl_i or negedge rst_ni) begin
      if (!rst_ni) sda_out_ptog  <= 1'b0;
      else if (sw_reset_i | !transmitting) sda_out_ptog  <= 1'b0;
      else sda_out_ptog  <= ddr_mode ^ sda_out_ptog;
    end
    // SCL-rising logic captures the new toggle bit and switches over to emitting the
    // odd-numbered bit.
    logic sda_out_ntog;
    always_ff @(posedge scl_ni or negedge rst_ni) begin
      if (!rst_ni) sda_out_ntog  <= 1'b0;
      else if (sw_reset_i | !transmitting) sda_out_ntog <= 1'b0;
      else sda_out_ntog  <= sda_out_ptog;
    end
    // Use the toggle mismatch to determine the clock phase, observing that in SDR mode only the
    // SCL-negedge bits are used (`sda_nq`).
    assign sda_o = (sda_out_ptog ^ sda_out_ntog) ? sda_pq[0][9] : sda_nq[0][9];
  end else begin : no_tokens
    // This approach just uses the clock state directly, but introduces a combinational path
    // from SCL to SDA.
    assign sda_o = scl_i ? sda_pq[0][9] : sda_nq[0][9];
  end

  // TODO: Request/response interface is incomplete at present; just capture the received word here
  // for checking.
  always_comb begin
    for (int unsigned t = 0; t < NumTargets; t++)
      trx_dready_o[t] = tx_starting;  // TODO: Select the appropriate target!
  end

  always_comb begin
    rsp = '0;
    if (state_q == State_RxCmdDDR || state_q == State_RxDataDDR) begin
      rsp.wdata = ddr_payload[15:0];
      rsp.wlen = 2'b01;
      rsp.dtype = (state_q == State_RxDataDDR) ? I3CDType_DataWord : I3CDType_CommandWord;
    end else if (state_q == State_RxCRCDDR) begin
      rsp.dtype = I3CDType_CRCWord;   
    end else begin
      rsp.wdata[7:0] = sda_pq[0][8:1];
      rsp.dtype = I3CDType_SDRBytes;
    end
    // TODO: Fold these into the response type; TCRI defines behavior?
    rsp.parity_err = parity_error;
    rsp.crc5_err   = crc5_error;
  end

  always_ff @(posedge scl_ni or negedge rst_ni) begin
    if (!rst_ni) begin
      trx_rtoggle_o <= 1'b0;
      trx_rsp_o     <= '0;
    end begin
      if (state_q == State_RxSDR     || state_q == State_RxCmdDDR ||
          state_q == State_RxDataDDR || state_q == State_RxCRCDDR) begin
        // Capture the SDR write data without the ACK, and HDR-DDR data without the parity bits,
        // because this information is required in the final cycle of SCL in preparation for sending
        // transmitting read data.
        if ((ddr_mode & prepen_bit) | (!ddr_mode & penult_bit)) trx_rsp_o <= rsp;
        // Present the information, if appropriate.
        // TODO: Everything is forwarded into the IP block for now.
        if (last_bit) trx_rtoggle_o <= !trx_rtoggle_o;
      end
    end
  end

endmodule
