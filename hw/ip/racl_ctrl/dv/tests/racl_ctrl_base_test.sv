// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

class racl_ctrl_base_test extends cip_base_test #(
    .CFG_T(racl_ctrl_env_cfg),
    .ENV_T(racl_ctrl_env)
  );

  `uvm_component_utils(racl_ctrl_base_test)
  `uvm_component_new

  // the base class dv_base_test creates the following instances:
  // racl_ctrl_env_cfg: cfg
  // racl_ctrl_env:     env

  // the base class also looks up UVM_TEST_SEQ plusarg to create and run that seq in
  // the run_phase; as such, nothing more needs to be done

endclass : racl_ctrl_base_test
