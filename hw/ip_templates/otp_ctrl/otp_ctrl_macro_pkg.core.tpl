CAPI=2:
# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

name: ${instance_vlnv("lowrisc:ip:otp_ctrl_macro_pkg:0.1")}
description: "Package with common interface definitions for OTP primitive."
virtual:
  - lowrisc:virtual_ip:otp_ctrl_macro_pkg

filesets:
  files_rtl:
    files:
      - rtl/otp_ctrl_macro_pkg.sv
    file_type: systemVerilogSource

  files_verilator_waiver:
    depend:
      # common waivers
      - lowrisc:lint:common

  files_ascentlint_waiver:
    depend:
      # common waivers
      - lowrisc:lint:common

  files_veriblelint_waiver:
    depend:
      # common waivers
      - lowrisc:lint:common

targets:
  default: &default_target
    filesets:
      - tool_verilator   ? (files_verilator_waiver)
      - tool_ascentlint  ? (files_ascentlint_waiver)
      - tool_veriblelint ? (files_veriblelint_waiver)
      - files_rtl

  lint:
    <<: *default_target
    default_tool: verilator
    parameters:
      - SYNTHESIS=true
    tools:
      verilator:
        mode: lint-only
        verilator_options:
          - "-Wall"
