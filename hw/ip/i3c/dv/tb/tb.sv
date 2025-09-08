// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Simple Verilator testbench for the I3C core.

`ifndef VERILATOR
`define STANDALONE
`endif

module tb(
`ifndef STANDALONE
  input clk_i,
  input rst_ni,

  input clk_aon_i,
  input rst_aon_ni
`endif
);

`ifdef STANDALONE
  logic clk_i;
  logic rst_ni;

  logic clk_aon_i;
  logic rst_aon_ni;
`endif

  import i3c_consts_pkg::*;
  import i3c_pkg::*;
  import i3c_reg_pkg::*;

  // Behavioural utilities to help with initial bring up.
  `include "../env/i3c_utils.svh"

  // Connect the Controller and Target together on a single I3C bus?
  localparam bit SingleBus = 1;

  // Number of SDA lines; presently, because HDR-BT is not supported, this must be 1.
  localparam int unsigned NumSDALanes = 1;

  // Properties of TL-UL interface.
  localparam int unsigned AW  = top_pkg::TL_AW;
  localparam int unsigned DW  = top_pkg::TL_DW;
  localparam int unsigned DBW = top_pkg::TL_DBW;

  // Simulation lifetime.
  logic [31:0] cnt;
  initial begin
    $display("I3C simulation starting.\nLogging to sim.fst\n");
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) cnt <= '0;
    else begin
      cnt <= cnt + 'b1;
      if (cnt[31]) $finish;
    end
  end

  `ifdef STANDALONE
  // Main clock to the IP block (50MHz).
  localparam int unsigned rst_period = 100 * 1000;
  localparam int unsigned clk_period = 20 * 1000;
  initial begin
    clk_i = 1'b1;
    rst_ni = 1'b0;
    #rst_period rst_ni = 1'b1;
  end
  always #(clk_period/2) clk_i = !clk_i;

  // Always On clock (100kHz).
  localparam int unsigned aon_rst_period = 1000 * 1000;
  localparam int unsigned aon_clk_period = 10 * 1000 * 1000;
  initial begin
    clk_aon_i = 1'b1;
    rst_aon_ni = 1'b0;
    #aon_rst_period rst_aon_ni = 1'b1;
  end
  always #(aon_clk_period/2) clk_aon_i = !clk_aon_i;
  `endif

  // Register offsets, indexed by name.
  int unsigned reg_offsets[string];
  // Message data to be sent over the I3C bus.
  byte msg_data[$];
  bit msg_bits[$];

  tlul_pkg::tl_h2d_t tl_h2d;
  tlul_pkg::tl_d2h_t tl_d2h;

  logic lsio_trigger;

  // Interrupts.
  // - master interrupt, asserted when any HCI interrupt is asserted.
  logic intr_hci;

  // I3C buses.
  wire ctrl_scl, targ_scl;
  wire [NumSDALanes-1:0] ctrl_sda, targ_sda;

  // I3C Controller driver signals.
  logic cio_ctrl_scl;
  logic cio_ctrl_scl_pp_en;
  logic cio_ctrl_scl_od_en;
  logic cio_ctrl_sda_pp_en;
  logic cio_ctrl_sda_od_en;
  logic [NumSDALanes-1:0] cio_ctrl_sda;

  // Pullup enables.
  logic cio_ctrl_scl_pu_en;
  logic cio_ctrl_sda_pu_en;

  // High-keeper enables.
  logic cio_scl_hk_en;
  logic cio_sda_hk_en;

  // I3C Target driver signals.
  logic cio_targ_scl;
  logic cio_targ_scl_pp_en;
  logic cio_targ_scl_od_en;
  logic cio_targ_sda_pp_en;
  logic cio_targ_sda_od_en;
  logic [NumSDALanes-1:0] cio_targ_sda;

  // Chip reset request.
  logic cio_chip_rst_req;

  // Use a single PHY if the Controller and Target are on a single bus?
  wire single_phy = SingleBus & 1'b1;

  prim_alert_pkg::alert_rx_t alert_rx;
  prim_alert_pkg::alert_tx_t alert_tx;
  assign alert_rx = '0;

  // Device Under Test.
  i3c dut(
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),

    .clk_aon_i            (clk_aon_i),
    .rst_aon_ni           (rst_aon_ni),

    // Register interface.
    .tl_i                 (tl_h2d),
    .tl_o                 (tl_d2h),

    // Alerts.
    .alert_rx_i           (alert_rx),
    .alert_tx_o           (alert_tx),

    // RACL interface.
    .racl_policies_i      (top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
    .racl_error_o         (),
    // I3C Controller I/O signaling.
    .cio_ctrl_scl_i       (ctrl_scl),
    .cio_ctrl_scl_o       (cio_ctrl_scl),
    .cio_ctrl_scl_pp_en_o (cio_ctrl_scl_pp_en),
    .cio_ctrl_scl_od_en_o (cio_ctrl_scl_od_en),
    .cio_ctrl_sda_i       (ctrl_sda),
    .cio_ctrl_sda_o       (cio_ctrl_sda),
    .cio_ctrl_sda_pp_en_o (cio_ctrl_sda_pp_en),
    .cio_ctrl_sda_od_en_o (cio_ctrl_sda_od_en),

    // Pullup enables for open drain intervals.
    .cio_ctrl_scl_pu_en_o (cio_ctrl_scl_pu_en),
    .cio_ctrl_sda_pu_en_o (cio_ctrl_sda_pu_en),

    // High-keeper enables.
    .cio_scl_hk_en_o      (cio_scl_hk_en),
    .cio_sda_hk_en_o      (cio_sda_hk_en),

    // I3C Target I/O signaling.
    .cio_targ_scl_i       (targ_scl),
    .cio_targ_sda_i       (targ_sda),
    .cio_targ_sda_o       (cio_targ_sda),
    .cio_targ_sda_pp_en_o (cio_targ_sda_pp_en),
    .cio_targ_sda_od_en_o (cio_targ_sda_od_en),

    // Chip reset request.
    .cio_chip_rst_req_o   (cio_chip_rst_req),

    // Interrupt-signaling to hardware
    .lsio_trigger_o       (lsio_trigger),

    // Interrupts.
    // - master interrupt, asserted when any HCI interrupt is asserted.
    .intr_hci_o           (intr_hci),

    // DFT-related controls.
    .ram_cfg_i            ('0),
    .ram_cfg_rsp_o        (),
    .dat_cfg_i            ('0),
    .dat_cfg_rsp_o        (),
    .dct_cfg_i            ('0),
    .dct_cfg_rsp_o        (),
    .mbist_en_i           (1'b0),
    .scan_clk_i           (1'b0),
    .scan_rst_ni          (1'b1),
    .scanmode_i           (prim_mubi_pkg::MuBi4False)
  );

  // Control signals to the I3C Controller PHY, which may be the only PHY.
  // This supports two modes of operation:
  // - Primary Controller (only; cannot become Secondary) with Target on the same bus.
  // - Primary/Secondary Controller, capable of switching roles.
  wire prim_phy_sda_pp_en = cio_ctrl_sda_pp_en | (single_phy & cio_targ_sda_pp_en);
  wire prim_phy_sda_od_en = cio_ctrl_sda_od_en | (single_phy & cio_targ_sda_od_en);
  // Each party has the capacity to drive the data line into the PHY low.
  wire prim_phy_ctrl_sda  = cio_ctrl_sda_pp_en ? cio_ctrl_sda : !(cio_ctrl_sda_od_en & !cio_ctrl_sda);
  wire prim_phy_targ_sda  = cio_targ_sda_pp_en ? cio_targ_sda : !(cio_targ_sda_od_en & !cio_targ_sda);
  // Data line is high only if both parties agree that it shall be high.
  wire prim_phy_sda       = prim_phy_ctrl_sda & (prim_phy_targ_sda | !single_phy);

  // I3C Controller driver; anticipated to be outside of the i3c IP block itself.
  i3c_phy #(
    .NumSDALanes  (NumSDALanes)
  ) u_ctrl_phy (
    // Pull-up enables.
    .scl_pu_en_i  (cio_ctrl_scl_pu_en),
    .sda_pu_en_i  (cio_ctrl_sda_pu_en),

    // SCL driver enables.
    .scl_pp_en_i  (cio_ctrl_scl_pp_en),
    .scl_od_en_i  (cio_ctrl_scl_od_en),
    // SCL signal from IP block.
    .scl_i        (cio_ctrl_scl),

    // SDA driver enables.
    .sda_pp_en_i  (prim_phy_sda_pp_en),
    .sda_od_en_i  (prim_phy_sda_od_en),
    // SDA signal from IP block.
    .sda_i        (prim_phy_sda),

    // I3C I/O signals.
    .scl          (ctrl_scl),
    .sda          (ctrl_sda)
  );

  // I3C Target driver; anticipated to be outside of the i3c IP block itself,
  // if present.
  // - This secondary PHY would be used in the event that the IP block is deployed as separated
  //   controller and target on two different I3C buses. In this case the Controller must remain
  //   as the Primary Controller and cannot hand over the bus to another controller.
  wire sec_phy_sda_pp_en = cio_targ_sda_pp_en & !single_phy;
  wire sec_phy_sda_od_en = cio_targ_sda_od_en & !single_phy;

  i3c_phy #(
    .NumSDALanes  (NumSDALanes)
  ) u_targ_phy (
    // Target does not drive pull-ups.
    .scl_pu_en_i  (1'b0),
    .sda_pu_en_i  (1'b0),

    // Target does not drive SCL line (no support for HDR-BT mode).
    .scl_pp_en_i  (1'b0),
    .scl_od_en_i  (1'b0),
    .scl_i        (1'b1),

    // SDA driver enables.
    .sda_pp_en_i  (sec_phy_sda_pp_en),
    .sda_od_en_i  (sec_phy_sda_od_en),
    // SDA signal from IP block.
    .sda_i        (cio_targ_sda),

    // I3C I/O signals.
    .scl          (targ_scl),
    .sda          (targ_sda)
  );

  // Optionally tie the I3C Controller and Target together on a single bus.
  assign targ_scl = ctrl_scl;
  assign targ_sda = ctrl_sda;


  // I3C high-keepers.
  // - expected to be external to the device, but if not then there must be some means of disabling
  //   them in the event that they are insufficient for the physical bus properties.
  `ifdef VERILATOR
  assign ctrl_scl = (cio_ctrl_scl_pp_en | cio_ctrl_scl_od_en) ? 1'bZ : cio_scl_hk_en;
  assign ctrl_sda = (cio_ctrl_sda_pp_en | cio_ctrl_sda_od_en) ? 1'bZ : cio_sda_hk_en;
  `else
  assign (weak0, weak1) ctrl_scl = cio_scl_hk_en ? 1'b1 : 1'bZ;
  assign (weak0, weak1) ctrl_sda = cio_sda_hk_en ? 1'b1 : 1'bZ;
  `endif

  // TODO: Move the following into an interface/module?

  // Host->device TL-UL with no integrity.
  tlul_pkg::tl_h2d_t tl_h2d_int;
  // Configuration file descriptor.
  int cfg;
  // Program the configuration registers.
  logic               cfg_prog;
  tlul_pkg::tl_a_op_e cfg_opcode;
  logic [AW-1:0]      cfg_addr;
  logic [DBW-1:0]     cfg_wmask;
  logic [DW-1:0]      cfg_wdata;
  wire  [AW-1:0]      cfg_addr_inc = (cfg_addr == 32'(I3C_XFER_DATA_PORT_OFFSET)) ? 'd0 : 'd4;
  always_comb begin
    tl_h2d_int = '0;
    // A channel carries write request.
    tl_h2d_int.a_valid   = cfg_prog;
    tl_h2d_int.a_opcode  = cfg_opcode;
    tl_h2d_int.a_param   = '0;
    tl_h2d_int.a_address = AW'(cfg_addr);
    tl_h2d_int.a_source  = '0;
    tl_h2d_int.a_size    = 'h2;
    tl_h2d_int.a_mask    = (cfg_opcode == tlul_pkg::Get) ? {DBW{1'b1}} : cfg_wmask;
    tl_h2d_int.a_data    = cfg_wdata;
    tl_h2d_int.a_user.instr_type = prim_mubi_pkg::MuBi4False;
    // D channel always accepts response immediately.
    tl_h2d_int.d_ready   = 1'b1;
  end

  tlul_cmd_intg_gen u_intg_gen(
    .tl_i (tl_h2d_int),
    .tl_o (tl_h2d)
  );

  // Open configuration file.
  initial begin
    static int test = 0;
    string cfg_filename;
    void'($value$plusargs("test=%x", test));
    case (test)
      0: cfg_filename = "cfg_regs";
      1: cfg_filename = "cfg_slow";
      2: cfg_filename = "cfg_hci";
      3: cfg_filename = "cfg_dxt";
      4: cfg_filename = "sdr_addr";
    endcase
    cfg_filename = {"scripts/", cfg_filename};
    cfg = $fopen(cfg_filename, "r");
    if (~|cfg) $display("WARNING: Unable to open config file '%s'", cfg_filename);
  end

  // Load data into the supplied queue
  function automatic int load_file(ref byte data[$], input string filename, int unsigned addr);
    localparam int EOF = -1;
    int msg;
    int ch;
    msg = $fopen(filename, "rb");
    if (~|msg) begin
      $display("WARNING: Unable to read message data file '%s'", filename);
    end else begin
      bit [1:0] parity = 2'b01;
      bit [4:0] crc5 = 5'h1f;
      bit msg_bits[$];
      do begin
        ch = $fgetc(msg);
        if (ch != EOF) begin
          // $display("%x", ch);
          data.push_back(ch[7:0]);
        end
      end while (ch != EOF);
      $fclose(msg);
      // We are presently sending 32-bit words, little endian order and the bits within those
      // words are sent big endian.
      $display("data size %d", data.size());
      for (int unsigned idx = 0; idx < data.size(); idx += 4) begin
        bit [31:0] w = {data[idx+3], data[idx+2], data[idx+1], data[idx]};
        // The bits are checked in the order of transmission, which means that we
        // need to know whether the data will be sent in HDR or SDR mode!
        for (int i = 31; i >= 0; i--) msg_bits.push_back(w[i]);  // SDR ordering.
      end
      // Update CRC-5 and Parity values.
      // TODO: In USB land we invert the final result?
      parity = update_parity(msg_bits, parity);
      crc5 = update_crc5(msg_bits, crc5);
      $display("CRC-5: 0x%x", crc5);
      $display("Parity: 0b%b", parity);
      return 0;
    end
    return -1;
  endfunction

  // Save data from the supplied queue to a file.
  function automatic int save_file(ref byte data[$], input string filename);
    int msg;
    msg = $fopen(filename, "wb");
    if (~|msg) begin
      $display("WARNING: Unable to save message data file '%s'", filename);
    end else begin
      for (int unsigned i = 0; i < data.size(); i++) begin
        $fwrite(msg, "%c", data[i]);
      end
      $fclose(msg);
    end
  endfunction

  // Load the symbolic names of the registers from the `i3c_reg_pkg` file to make the configuration
  // scripts less fragile.
  function automatic void load_reg_names(string filename);
    int fd;
    fd = $fopen(filename, "r");
    if (~|fd) begin
      $display("WARNING: Unable to load register names from '%s'", filename);
    end else begin
      string pattern = "  parameter logic [BlockAw-1:0] I3C_";
      int lineno = 0;
      string str;
      while (!$feof(fd)) begin
        int len = $fgets(str, fd);
        if (len <= 0) break;
        lineno++;
        if (len >= 36 && 0 == pattern.compare(str.substr(0, 35))) begin
          int unsigned offset;
          string name;
          // $display("Line %d: %s", lineno, str);
          void'($sscanf(str, "  parameter logic [BlockAw-1:0] I3C_%s = 12'h %x;", name, offset));
          $display("Reg: '%s' at offset 0x%x", name, offset);
          reg_offsets[name] = offset;
        end
      end
      $fclose(fd);
    end
  endfunction

  // ASCII characters used in the script parsing.
  localparam byte Char_LF = 10;
  localparam byte Char_Hash = 35;
  localparam byte Char_Slash = 47;
  localparam byte Char_0 = 48;
  localparam byte Char_9 = 57;
  localparam byte Char_Colon = 58;
  localparam byte Char_A = 65;
  localparam byte Char_C = 67;
  localparam byte Char_F = 70;
  localparam byte Char_L = 76;
  localparam byte Char_R = 82;
  localparam byte Char_S = 83;
  localparam byte Char_X = 88;
  localparam byte Char_a = 97;
  localparam byte Char_f = 102;
  localparam byte Char_x = 120;

  function automatic bit [3:0] hexdig(byte ch);
    return (ch >= Char_0 && ch <= Char_9) ? 4'(ch - Char_0) : 4'((ch & ~8'h20) + 8'd10 - Char_A);
  endfunction

  function automatic bit isxdigit(byte ch);
    return |{ch >= Char_A && ch <= Char_F,
             ch >= Char_a && ch <= Char_f,
             ch >= Char_0 && ch <= Char_9};
  endfunction

  function automatic int unsigned get_reg_offset(inout string s);
    int unsigned idx = 0;
    int unsigned n = 0;
    while (isxdigit(s[idx])) begin
      n = {n[27:0], hexdig(s[idx])};
      idx++;
    end
    case (s[idx])
      Char_LF: /* Do nothing */;
      Char_Colon: idx = idx + 1;
      default: begin
        string name;
        // Scan until we reach any colon following the register name.
        while (idx < s.len() && s[idx] != Char_LF && s[idx] != Char_Colon) idx++;
        name = {s.substr(0, idx - 1), "_OFFSET"};
        n = reg_offsets[name];
        $display("Looked up name '%s' -> 0x%x", name, n);
        // Skip past the colon.
        if (idx < s.len() && s[idx] == Char_Colon) idx++;
      end
    endcase
    // Remove the parsed characters.
    s = s.substr(idx, s.len() - 1);
    // Return the register offset.
    // $display("n: 0x%x", n);
    return n;
  endfunction

  string save_filename;
  int unsigned chk_values[$];
  int unsigned save_len_req;
  int unsigned save_len;
  logic [31:0] delay;
  logic proceed;
  logic loading;
  logic saving;

  initial begin
    load_reg_names("../../rtl/i3c_reg_pkg.sv");
  end

  // Advance the configuration logic; parses test script and performs the following operations:
  // - write to register/memory.
  // - read from register/memory.
  // - checked read from register/memory.
  // - delay for a specified number of cycles.
  // - load file contents into register/memory.
  // - save register/memory contents to file.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cfg_prog  <= 1'b0;
      delay     <= '0;
      loading   <= 1'b0;
      proceed   <= 1'b1;
      saving    <= 1'b0;
      save_len  <= '0;
    end else begin
      if (proceed || (tl_h2d.a_valid & tl_d2h.a_ready)) begin
        if (loading) begin
          if (0 != msg_data.size()) begin : load_data
            logic [top_pkg::TL_DW-1:0] d;
            d[7:0]   = msg_data.pop_front();
            d[15:8]  = msg_data.pop_front();
            d[23:16] = msg_data.pop_front();
            d[31:24] = msg_data.pop_front();
            // $display("Loading %x %x", d, cfg_addr + 4);
            // Issue write request.
            cfg_prog    <= 1'b1;
            cfg_addr    <= cfg_addr + cfg_addr_inc;
            cfg_opcode  <= tlul_pkg::PutFullData;
            cfg_wmask   <= '1;
            cfg_wdata   <= d;
            proceed     <= 1'b0;
          end else begin
            cfg_prog    <= 1'b0;
            loading     <= 1'b0;
            proceed     <= 1'b1;
          end
        end else if (saving) begin
          if (save_len_req != 0) begin
            // Issue read request; returned read data is collected later.
            cfg_prog      <= 1'b1;
            cfg_opcode    <= tlul_pkg::Get;
            cfg_addr      <= cfg_addr + cfg_addr_inc;
            cfg_wmask     <= '1;
            cfg_wdata     <= '0;
            save_len_req  <= save_len_req - 4;
            proceed       <= 1'b0;
          end else begin
            // Do not deassert `saving` until we have received all responses.
            cfg_prog    <= 1'b0;
            proceed     <= 1'b1;
          end
        end else if (|cfg && $feof(cfg)) begin
          cfg_prog <= 1'b0;
        end else begin : read_config
          if (~|delay) begin
            string str;
            if (0 < $fgets(str, cfg)) begin
              case (str[0])
                // Time delay
                Char_Hash: begin : read_delay
                  int unsigned cycles;
                  void'($sscanf(str, "#%d", cycles));
                  $display("Delaying for %d cycle(s)", cycles);
                  delay <= cycles;
                  proceed <= 1'b1;
                  cfg_prog <= 1'b0;
                end
                // Comments
                Char_Slash: begin
                  proceed <= 1'b1;
                  cfg_prog <= 1'b0;
                end
                // Load file
                Char_L: begin
                  int unsigned addr;
                  string filename;
                  str = str.substr(2, str.len() - 1);
                  addr = get_reg_offset(str);
                  void'($sscanf(str, "%s", filename));
                  $display("Loading file '%s' at 0x%x", filename, addr);
                  void'(load_file(msg_data, filename, addr));
                  // When loading, the address is incremented before use, except for XFER_DATA_PORT.
                  cfg_addr <= addr - ((addr == 32'(I3C_XFER_DATA_PORT_OFFSET)) ? 'd0 : 'd4);
                  loading <= (msg_data.size() != 0);
                  proceed <= 1'b1;
                end
                // Save file
                Char_S: begin : save_file
                  int unsigned addr;
                  int unsigned len;
                  str = str.substr(2, str.len() - 1);
                  addr = get_reg_offset(str);
                  void'($sscanf(str, "%x:%s", len, save_filename));
                  $display("Saving file '%s' of %x byte(s)", save_filename, len);
                  // TODO: We handle only complete 32-bit words at present.
                  save_len_req <= len;
                  save_len  <= len - 4;  // Used before decrement.
                  // When saving, the address is incremented before use, except for XFER_DATA_PORT.
                  cfg_addr  <= addr - ((addr == 32'(I3C_XFER_DATA_PORT_OFFSET)) ? 'd0 : 'd4);
                  saving    <= (len != 0);
                  proceed   <= 1'b1;
                end
                // Terminate simulation.
                Char_X: begin
                  $fclose(cfg);
                  $display("Terminating at end of configuration script");
                  $finish;
                end
                // Register read/write
                default: begin : reg_op
                  int unsigned addr, wmask, wdata;
                  tlul_pkg::tl_a_op_e opcode;
                  string name;
                  case (str[0])
                    // Check register value.
                    Char_C: begin
                      int unsigned chk_val;
                      str = str.substr(2, str.len() - 1);
                      addr = get_reg_offset(str);
                      void'($sscanf(str, "%x", chk_val));
                      chk_values.push_back(chk_val);
                      opcode = tlul_pkg::Get;
                    end
                    // Read register.
                    Char_R: begin
                      str = str.substr(2, str.len() - 1);
                      addr = get_reg_offset(str);
                      opcode = tlul_pkg::Get;
                    end
                    default: begin
                      str = str.substr(2, str.len() - 1);
                      addr = get_reg_offset(str);
                      void'($sscanf(str, "%x:%x", wmask, wdata));
                      opcode = &wmask[top_pkg::TL_DBW-1:0] ? tlul_pkg::PutFullData
                                                           : tlul_pkg::PutPartialData;
                    end
                  endcase
                  case (opcode)
                    tlul_pkg::PutFullData: $display("Write [%x]:%x", addr, wdata);
                    tlul_pkg::PutPartialData: $display("Write [%x]:%x,%x", addr, wmask, wdata);
                    default: $display("Read [%x]", addr);
                  endcase
                  cfg_prog    <= 1'b1;
                  cfg_addr    <= addr;
                  cfg_opcode  <= tlul_pkg::tl_a_op_e'(opcode);
                  cfg_wmask   <= wmask[top_pkg::TL_DBW-1:0];
                  cfg_wdata   <= wdata;
                  // We must wait until the transfer has completed.
                  proceed     <= 1'b0;
                end
              endcase
            end else begin
              // All done.
              cfg_prog <= 1'b0;
              proceed  <= 1'b0;
            end
          end else delay <= delay - 1'b1;
        end
      end
      // Returned data from the DUT.
      if (tl_d2h.d_valid & tl_h2d.d_ready & tl_d2h.d_opcode == tlul_pkg::AccessAckData) begin
        if (saving) begin
          msg_data.push_back(tl_d2h.d_data[7:0]);
          msg_data.push_back(tl_d2h.d_data[15:8]);
          msg_data.push_back(tl_d2h.d_data[23:16]);
          msg_data.push_back(tl_d2h.d_data[31:24]);
          // Have we yet captured enough data?
          if (~|save_len) begin
            void'(save_file(msg_data, save_filename));
            $display("File '%s' saved", save_filename);
            msg_data.delete();
            saving <= 1'b0;
          end begin
            save_len <= save_len - 4;
          end
        end else if (0 != chk_values.size()) begin : chk_read
          logic [DW-1:0] exp_val, act_val;
          exp_val = chk_values.pop_front();
          act_val = tl_d2h.d_data;
          if (act_val !== exp_val) begin
            $display("ERROR: Actual value 0x%x did not match expected value 0x%x", act_val, exp_val);
            $finish;
          end
        end
      end
    end
  end

endmodule
