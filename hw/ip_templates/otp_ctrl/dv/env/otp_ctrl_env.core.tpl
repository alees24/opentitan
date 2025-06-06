CAPI=2:
# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
name: ${instance_vlnv("lowrisc:dv:otp_ctrl_env:0.1")}
description: "OTP_CTRL DV UVM environment"

filesets:
  files_dv:
    depend:
      - lowrisc:dv:ralgen
      - lowrisc:dv:cip_lib
      - lowrisc:dv:mem_bkdr_util
      - lowrisc:dv:crypto_dpi_present
      - lowrisc:dv:lc_ctrl_dv_utils
      - lowrisc:ip:otp_macro_pkg
      - lowrisc:dv:otp_macro_env
    files:
      - otp_scrambler_pkg.sv
      - otp_ctrl_env_pkg.sv
      - otp_ctrl_if.sv
      - otp_ctrl_ast_inputs_cfg.sv: {is_include_file: true}
      - otp_ctrl_env_cfg.sv: {is_include_file: true}
      - otp_ctrl_env_cov.sv: {is_include_file: true}
      - otp_ctrl_virtual_sequencer.sv: {is_include_file: true}
      - otp_ctrl_scoreboard.sv: {is_include_file: true}
      - otp_ctrl_env.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_vseq_list.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_callback_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_base_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_common_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_wake_up_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_smoke_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_partition_walk_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_low_freq_read_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_init_fail_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_dai_lock_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_dai_errs_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_macro_errs_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_background_chks_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_check_fail_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_parallel_base_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_regwen_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_parallel_key_req_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_parallel_lc_req_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_parallel_lc_esc_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_test_access_vseq.sv: {is_include_file: true}
      - seq_lib/otp_ctrl_stress_all_vseq.sv: {is_include_file: true}
    file_type: systemVerilogSource

generate:
  ral:
    generator: ralgen
    parameters:
      name: otp_ctrl
      ip_hjson: ../../data/otp_ctrl.hjson
    position: prepend

targets:
  default:
    filesets:
      - files_dv
    generate:
      - ral
