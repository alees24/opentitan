CAPI=2:
# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
name: ${instance_vlnv(f"lowrisc:dv:{module_instance_name}_env:0.1")}
description: "${module_instance_name.upper()} DV UVM environment"
filesets:
  files_dv:
    depend:
      - lowrisc:dv:ralgen
      - lowrisc:dv:cip_lib
      - ${instance_vlnv(f"lowrisc:ip:{module_instance_name}:0.1")}
      - lowrisc:dv:pwm_monitor
    files:
      - pwm_env_pkg.sv
      - pwm_env_cfg.sv: {is_include_file: true}
      - pwm_env_cov.sv: {is_include_file: true}
      - pwm_virtual_sequencer.sv: {is_include_file: true}
      - pwm_scoreboard.sv: {is_include_file: true}
      - pwm_env.sv: {is_include_file: true}
      - seq_lib/pwm_vseq_list.sv: {is_include_file: true}
      - seq_lib/pwm_base_vseq.sv: {is_include_file: true}
      - seq_lib/pwm_common_vseq.sv: {is_include_file: true}
      - seq_lib/pwm_phase_vseq.sv: {is_include_file: true}
      - seq_lib/pwm_smoke_vseq.sv: {is_include_file: true}
      - seq_lib/pwm_rand_output_vseq.sv: {is_include_file: true}
      - seq_lib/pwm_heartbeat_wrap_vseq.sv: {is_include_file: true}
      - seq_lib/pwm_perf_vseq.sv: {is_include_file: true}
      - seq_lib/pwm_regwen_vseq.sv: {is_include_file: true}
      - seq_lib/pwm_stress_all_vseq.sv: {is_include_file: true}
    file_type: systemVerilogSource

generate:
  ral:
    generator: ralgen
    parameters:
      name: pwm
      ip_hjson: ../../data/pwm.hjson
    position: prepend

targets:
  default:
    filesets:
      - files_dv
    generate:
      - ral
