# GPIO Checklist

This checklist is for [Hardware Stage](../../../../../doc/project_governance/development_stages.md) transitions for the [GPIO peripheral][GPIO Spec].
All checklist items refer to the content in the [Checklist.](../../../../../doc/project_governance/checklist/README.md)

## Design Checklist

### D1

Type          | Item                           | Resolution  | Note/Collaterals
--------------|--------------------------------|-------------|------------------
Documentation | [SPEC_COMPLETE][]              | Done        | [GPIO Spec][]
Documentation | [CSR_DEFINED][]                | Done        | [GPIO CSR][]
RTL           | [CLKRST_CONNECTED][]           | Done        |
RTL           | [IP_TOP][]                     | Done        |
RTL           | [IP_INSTANTIABLE][]            | Done        |
RTL           | [PHYSICAL_MACROS_DEFINED_80][] | N/A         |
RTL           | [FUNC_IMPLEMENTED][]           | Done        |
RTL           | [ASSERT_KNOWN_ADDED][]         | Done        |
Code Quality  | [LINT_SETUP][]                 | Done        |

[GPIO Spec]: ../
[GPIO CSR]: ../data/gpio.hjson


[SPEC_COMPLETE]:      ../../../../../doc/project_governance/checklist/README.md#spec_complete
[CSR_DEFINED]:        ../../../../../doc/project_governance/checklist/README.md#csr_defined
[CLKRST_CONNECTED]:   ../../../../../doc/project_governance/checklist/README.md#clkrst_connected
[IP_TOP]:             ../../../../../doc/project_governance/checklist/README.md#ip_top
[IP_INSTANTIABLE]:    ../../../../../doc/project_governance/checklist/README.md#ip_instantiable
[PHYSICAL_MACROS_DEFINED_80]:   ../../../../../doc/project_governance/checklist/README.md#physical_macros_defined_80
[FUNC_IMPLEMENTED]:   ../../../../../doc/project_governance/checklist/README.md#func_implemented
[ASSERT_KNOWN_ADDED]: ../../../../../doc/project_governance/checklist/README.md#assert_known_added
[LINT_SETUP]:         ../../../../../doc/project_governance/checklist/README.md#lint_setup

### D2

Type          | Item                      | Resolution  | Note/Collaterals
--------------|---------------------------|-------------|------------------
Documentation | [NEW_FEATURES][]          | N/A         |
Documentation | [BLOCK_DIAGRAM][]         | N/A         |
Documentation | [DOC_INTERFACE][]         | Done        |
Documentation | [DOC_INTEGRATION_GUIDE][] | Waived      | This checklist item has been added retrospectively.
Documentation | [MISSING_FUNC][]          | N/A         |
Documentation | [FEATURE_FROZEN][]        | Done        |
RTL           | [FEATURE_COMPLETE][]      | Done        |
RTL           | [PORT_FROZEN][]           | Done        |
RTL           | [ARCHITECTURE_FROZEN][]   | Done        |
RTL           | [REVIEW_TODO][]           | Done        |
RTL           | [STYLE_X][]               | Done        |
RTL           | [CDC_SYNCMACRO][]         | N/A         |
Code Quality  | [LINT_PASS][]             | Done        |
Code Quality  | [CDC_SETUP][]             | Waived      | No block-level flow available - waived to top-level signoff.
Code Quality  | [RDC_SETUP][]             | Waived      | No block-level flow available - waived to top-level signoff.
Code Quality  | [AREA_CHECK][]            | Done        |
Code Quality  | [TIMING_CHECK][]          | Done        | Fmax 50MHz on NexysVideo
Security      | [SEC_CM_DOCUMENTED][]     | N/A         |

[NEW_FEATURES]:          ../../../../../doc/project_governance/checklist/README.md#new_features
[BLOCK_DIAGRAM]:         ../../../../../doc/project_governance/checklist/README.md#block_diagram
[DOC_INTERFACE]:         ../../../../../doc/project_governance/checklist/README.md#doc_interface
[DOC_INTEGRATION_GUIDE]: ../../../../../doc/project_governance/checklist/README.md#doc_integration_guide
[MISSING_FUNC]:          ../../../../../doc/project_governance/checklist/README.md#missing_func
[FEATURE_FROZEN]:        ../../../../../doc/project_governance/checklist/README.md#feature_frozen
[FEATURE_COMPLETE]:      ../../../../../doc/project_governance/checklist/README.md#feature_complete
[PORT_FROZEN]:           ../../../../../doc/project_governance/checklist/README.md#port_frozen
[ARCHITECTURE_FROZEN]:   ../../../../../doc/project_governance/checklist/README.md#architecture_frozen
[REVIEW_TODO]:           ../../../../../doc/project_governance/checklist/README.md#review_todo
[STYLE_X]:               ../../../../../doc/project_governance/checklist/README.md#style_x
[CDC_SYNCMACRO]:         ../../../../../doc/project_governance/checklist/README.md#cdc_syncmacro
[LINT_PASS]:             ../../../../../doc/project_governance/checklist/README.md#lint_pass
[CDC_SETUP]:             ../../../../../doc/project_governance/checklist/README.md#cdc_setup
[RDC_SETUP]:             ../../../../../doc/project_governance/checklist/README.md#rdc_setup
[AREA_CHECK]:            ../../../../../doc/project_governance/checklist/README.md#area_check
[TIMING_CHECK]:          ../../../../../doc/project_governance/checklist/README.md#timing_check
[SEC_CM_DOCUMENTED]:     ../../../../../doc/project_governance/checklist/README.md#sec_cm_documented

### D2S

 Type         | Item                         | Resolution  | Note/Collaterals
--------------|------------------------------|-------------|------------------
Security      | [SEC_CM_ASSETS_LISTED][]     | Done        |
Security      | [SEC_CM_IMPLEMENTED][]       | Done        |
Security      | [SEC_CM_RND_CNST][]          | N/A         |
Security      | [SEC_CM_NON_RESET_FLOPS][]   | N/A         |
Security      | [SEC_CM_SHADOW_REGS][]       | N/A         |
Security      | [SEC_CM_RTL_REVIEWED][]      | N/A         |
Security      | [SEC_CM_COUNCIL_REVIEWED][]  | N/A         | This block only contains the bus-integrity CM.

[SEC_CM_ASSETS_LISTED]:    ../../../../../doc/project_governance/checklist/README.md#sec_cm_assets_listed
[SEC_CM_IMPLEMENTED]:      ../../../../../doc/project_governance/checklist/README.md#sec_cm_implemented
[SEC_CM_RND_CNST]:         ../../../../../doc/project_governance/checklist/README.md#sec_cm_rnd_cnst
[SEC_CM_NON_RESET_FLOPS]:  ../../../../../doc/project_governance/checklist/README.md#sec_cm_non_reset_flops
[SEC_CM_SHADOW_REGS]:      ../../../../../doc/project_governance/checklist/README.md#sec_cm_shadow_regs
[SEC_CM_RTL_REVIEWED]:     ../../../../../doc/project_governance/checklist/README.md#sec_cm_rtl_reviewed
[SEC_CM_COUNCIL_REVIEWED]: ../../../../../doc/project_governance/checklist/README.md#sec_cm_council_reviewed

### D3

 Type         | Item                    | Resolution  | Note/Collaterals
--------------|-------------------------|-------------|------------------
Documentation | [NEW_FEATURES_D3][]     | N/A         |
RTL           | [TODO_COMPLETE][]       | Done        |
Code Quality  | [LINT_COMPLETE][]       | Done        |
Code Quality  | [CDC_COMPLETE][]        | Waived      | No block-level flow available - waived to top-level signoff
Code Quality  | [RDC_COMPLETE][]        | Waived      | No block-level flow available - waived to top-level signoff
Review        | [REVIEW_RTL][]          | Done        |
Review        | [REVIEW_DELETED_FF][]   | Waived      | No block-level flow available - waived to top-level signoff
Review        | [REVIEW_SW_CHANGE][]    | Done        |
Review        | [REVIEW_SW_ERRATA][]    | Done        |
Review        | Reviewer(s)             | Done        | eunchan@ jeoong@ weicai@ alphan@
Review        | Signoff date            | Done        | 2022-05-26

[NEW_FEATURES_D3]:      ../../../../../doc/project_governance/checklist/README.md#new_features_d3
[TODO_COMPLETE]:        ../../../../../doc/project_governance/checklist/README.md#todo_complete
[LINT_COMPLETE]:        ../../../../../doc/project_governance/checklist/README.md#lint_complete
[CDC_COMPLETE]:         ../../../../../doc/project_governance/checklist/README.md#cdc_complete
[RDC_COMPLETE]:         ../../../../../doc/project_governance/checklist/README.md#rdc_complete
[REVIEW_RTL]:           ../../../../../doc/project_governance/checklist/README.md#review_rtl
[REVIEW_DELETED_FF]:    ../../../../../doc/project_governance/checklist/README.md#review_deleted_ff
[REVIEW_SW_CHANGE]:     ../../../../../doc/project_governance/checklist/README.md#review_sw_change
[REVIEW_SW_ERRATA]:     ../../../../../doc/project_governance/checklist/README.md#review_sw_errata

## Verification Checklist

### V1

 Type         | Item                                  | Resolution      | Note/Collaterals
--------------|---------------------------------------|-----------------|------------------
Documentation | [DV_DOC_DRAFT_COMPLETED][]            | Done            | [gpio_dv_doc][]
Documentation | [TESTPLAN_COMPLETED][]                | Done            |
Testbench     | [TB_TOP_CREATED][]                    | Done            |
Testbench     | [PRELIMINARY_ASSERTION_CHECKS_ADDED][]| Done            |
Testbench     | [SIM_TB_ENV_CREATED][]                | Done            |
Testbench     | [SIM_RAL_MODEL_GEN_AUTOMATED][]       | Done            |
Testbench     | [CSR_CHECK_GEN_AUTOMATED][]           | waived          | Revisit later. Tool setup in progress.
Testbench     | [TB_GEN_AUTOMATED][]                  | N/A             |
Tests         | [SIM_SMOKE_TEST_PASSING][]            | Done            |
Tests         | [SIM_CSR_MEM_TEST_SUITE_PASSING][]    | Done            |
Tests         | [FPV_MAIN_ASSERTIONS_PROVEN][]        | N/A             |
Tool Setup    | [SIM_ALT_TOOL_SETUP][]                | Done            |
Regression    | [SIM_SMOKE_REGRESSION_SETUP][]        | Done w/ waivers | Exception (implemented in local)
Regression    | [SIM_NIGHTLY_REGRESSION_SETUP][]      | Done w/ waivers | Exception (implemented in local)
Regression    | [FPV_REGRESSION_SETUP][]              | N/A             |
Coverage      | [SIM_COVERAGE_MODEL_ADDED][]          | Done            |
Code Quality  | [TB_LINT_SETUP][]                     | Done            |
Integration   | [PRE_VERIFIED_SUB_MODULES_V1][]       | N/A             |
Review        | [DESIGN_SPEC_REVIEWED][]              | Done            |
Review        | [TESTPLAN_REVIEWED][]                 | Done            |
Review        | [STD_TEST_CATEGORIES_PLANNED][]       | Done            | Exception (Security, Power, Debug)
Review        | [V2_CHECKLIST_SCOPED][]               | Done            |

[gpio_dv_doc]: ../dv/README.md

[DV_DOC_DRAFT_COMPLETED]:             ../../../../../doc/project_governance/checklist/README.md#dv_doc_draft_completed
[TESTPLAN_COMPLETED]:                 ../../../../../doc/project_governance/checklist/README.md#testplan_completed
[TB_TOP_CREATED]:                     ../../../../../doc/project_governance/checklist/README.md#tb_top_created
[PRELIMINARY_ASSERTION_CHECKS_ADDED]: ../../../../../doc/project_governance/checklist/README.md#preliminary_assertion_checks_added
[SIM_TB_ENV_CREATED]:                 ../../../../../doc/project_governance/checklist/README.md#sim_tb_env_created
[SIM_RAL_MODEL_GEN_AUTOMATED]:        ../../../../../doc/project_governance/checklist/README.md#sim_ral_model_gen_automated
[CSR_CHECK_GEN_AUTOMATED]:            ../../../../../doc/project_governance/checklist/README.md#csr_check_gen_automated
[TB_GEN_AUTOMATED]:                   ../../../../../doc/project_governance/checklist/README.md#tb_gen_automated
[SIM_SMOKE_TEST_PASSING]:             ../../../../../doc/project_governance/checklist/README.md#sim_smoke_test_passing
[SIM_CSR_MEM_TEST_SUITE_PASSING]:     ../../../../../doc/project_governance/checklist/README.md#sim_csr_mem_test_suite_passing
[FPV_MAIN_ASSERTIONS_PROVEN]:         ../../../../../doc/project_governance/checklist/README.md#fpv_main_assertions_proven
[SIM_ALT_TOOL_SETUP]:                 ../../../../../doc/project_governance/checklist/README.md#sim_alt_tool_setup
[SIM_SMOKE_REGRESSION_SETUP]:         ../../../../../doc/project_governance/checklist/README.md#sim_smoke_regression_setup
[SIM_NIGHTLY_REGRESSION_SETUP]:       ../../../../../doc/project_governance/checklist/README.md#sim_nightly_regression_setup
[FPV_REGRESSION_SETUP]:               ../../../../../doc/project_governance/checklist/README.md#fpv_regression_setup
[SIM_COVERAGE_MODEL_ADDED]:           ../../../../../doc/project_governance/checklist/README.md#sim_coverage_model_added
[TB_LINT_SETUP]:                      ../../../../../doc/project_governance/checklist/README.md#tb_lint_setup
[PRE_VERIFIED_SUB_MODULES_V1]:        ../../../../../doc/project_governance/checklist/README.md#pre_verified_sub_modules_v1
[DESIGN_SPEC_REVIEWED]:               ../../../../../doc/project_governance/checklist/README.md#design_spec_reviewed
[TESTPLAN_REVIEWED]:                  ../../../../../doc/project_governance/checklist/README.md#testplan_reviewed
[STD_TEST_CATEGORIES_PLANNED]:        ../../../../../doc/project_governance/checklist/README.md#std_test_categories_planned
[V2_CHECKLIST_SCOPED]:                ../../../../../doc/project_governance/checklist/README.md#v2_checklist_scoped

### V2

 Type         | Item                                    | Resolution  | Note/Collaterals
--------------|-----------------------------------------|-------------|------------------
Documentation | [DESIGN_DELTAS_CAPTURED_V2][]           | N/A         |
Documentation | [DV_DOC_COMPLETED][]                    | Done        |
Testbench     | [FUNCTIONAL_COVERAGE_IMPLEMENTED][]     | Done        |
Testbench     | [ALL_INTERFACES_EXERCISED][]            | Done        |
Testbench     | [ALL_ASSERTION_CHECKS_ADDED][]          | Done        |
Testbench     | [SIM_TB_ENV_COMPLETED][]                | Done        |
Tests         | [SIM_ALL_TESTS_PASSING][]               | Done        | Resolved: [#680][]
Tests         | [FPV_ALL_ASSERTIONS_WRITTEN][]          | N/A         |
Tests         | [FPV_ALL_ASSUMPTIONS_REVIEWED][]        | N/A         |
Tests         | [SIM_FW_SIMULATED][]                    | N/A         |
Regression    | [SIM_NIGHTLY_REGRESSION_V2][]           | Done        |
Coverage      | [SIM_CODE_COVERAGE_V2][]                | Done        |
Coverage      | [SIM_FUNCTIONAL_COVERAGE_V2][]          | Done        | Resolved: [#807][]
Coverage      | [FPV_CODE_COVERAGE_V2][]                | N/A         |
Coverage      | [FPV_COI_COVERAGE_V2][]                 | N/A         |
Integration   | [PRE_VERIFIED_SUB_MODULES_V2][]         | N/A         |
Issues        | [NO_HIGH_PRIORITY_ISSUES_PENDING][]     | Done        |
Issues        | [ALL_LOW_PRIORITY_ISSUES_ROOT_CAUSED][] | Done        | [#41][] Not quite related, [#45][] root caused
Review        | [DV_DOC_TESTPLAN_REVIEWED][]            | Done        |
Review        | [V3_CHECKLIST_SCOPED][]                 | Done        |

[#41]: https://github.com/lowRISC/opentitan/issues/41
[#45]: https://github.com/lowRISC/opentitan/issues/45
[#680]: https://github.com/lowRISC/opentitan/pull/680
[#807]: https://github.com/lowRISC/opentitan/pull/807

[DESIGN_DELTAS_CAPTURED_V2]:          ../../../../../doc/project_governance/checklist/README.md#design_deltas_captured_v2
[DV_DOC_COMPLETED]:                   ../../../../../doc/project_governance/checklist/README.md#dv_doc_completed
[FUNCTIONAL_COVERAGE_IMPLEMENTED]:    ../../../../../doc/project_governance/checklist/README.md#functional_coverage_implemented
[ALL_INTERFACES_EXERCISED]:           ../../../../../doc/project_governance/checklist/README.md#all_interfaces_exercised
[ALL_ASSERTION_CHECKS_ADDED]:         ../../../../../doc/project_governance/checklist/README.md#all_assertion_checks_added
[SIM_TB_ENV_COMPLETED]:               ../../../../../doc/project_governance/checklist/README.md#sim_tb_env_completed
[SIM_ALL_TESTS_PASSING]:              ../../../../../doc/project_governance/checklist/README.md#sim_all_tests_passing
[FPV_ALL_ASSERTIONS_WRITTEN]:         ../../../../../doc/project_governance/checklist/README.md#fpv_all_assertions_written
[FPV_ALL_ASSUMPTIONS_REVIEWED]:       ../../../../../doc/project_governance/checklist/README.md#fpv_all_assumptions_reviewed
[SIM_FW_SIMULATED]:                   ../../../../../doc/project_governance/checklist/README.md#sim_fw_simulated
[SIM_NIGHTLY_REGRESSION_V2]:          ../../../../../doc/project_governance/checklist/README.md#sim_nightly_regression_v2
[SIM_CODE_COVERAGE_V2]:               ../../../../../doc/project_governance/checklist/README.md#sim_code_coverage_v2
[SIM_FUNCTIONAL_COVERAGE_V2]:         ../../../../../doc/project_governance/checklist/README.md#sim_functional_coverage_v2
[FPV_CODE_COVERAGE_V2]:               ../../../../../doc/project_governance/checklist/README.md#fpv_code_coverage_v2
[FPV_COI_COVERAGE_V2]:                ../../../../../doc/project_governance/checklist/README.md#fpv_coi_coverage_v2
[PRE_VERIFIED_SUB_MODULES_V2]:        ../../../../../doc/project_governance/checklist/README.md#pre_verified_sub_modules_v2
[NO_HIGH_PRIORITY_ISSUES_PENDING]:    ../../../../../doc/project_governance/checklist/README.md#no_high_priority_issues_pending
[ALL_LOW_PRIORITY_ISSUES_ROOT_CAUSED]:../../../../../doc/project_governance/checklist/README.md#all_low_priority_issues_root_caused
[DV_DOC_TESTPLAN_REVIEWED]:           ../../../../../doc/project_governance/checklist/README.md#dv_doc_testplan_reviewed
[V3_CHECKLIST_SCOPED]:                ../../../../../doc/project_governance/checklist/README.md#v3_checklist_scoped

### V2S

 Type         | Item                                    | Resolution  | Note/Collaterals
--------------|-----------------------------------------|-------------|------------------
Documentation | [SEC_CM_TESTPLAN_COMPLETED][]           | Done        |
Tests         | [FPV_SEC_CM_VERIFIED][]                 | N/A         |
Tests         | [SIM_SEC_CM_VERIFIED][]                 | Done        |
Coverage      | [SIM_COVERAGE_REVIEWED][]               | Done        |
Review        | [SEC_CM_DV_REVIEWED][]                  | Done        | Waived the V2S review meeting, since only 1 standard sec_cm - bus integrity.

[SEC_CM_TESTPLAN_COMPLETED]:          ../../../../../doc/project_governance/checklist/README.md#sec_cm_testplan_completed
[FPV_SEC_CM_VERIFIED]:                ../../../../../doc/project_governance/checklist/README.md#fpv_sec_cm_verified
[SIM_SEC_CM_VERIFIED]:                ../../../../../doc/project_governance/checklist/README.md#sim_sec_cm_verified
[SIM_COVERAGE_REVIEWED]:              ../../../../../doc/project_governance/checklist/README.md#sim_coverage_reviewed
[SEC_CM_DV_REVIEWED]:                 ../../../../../doc/project_governance/checklist/README.md#sec_cm_dv_reviewed

### V3

 Type         | Item                              | Resolution  | Note/Collaterals
--------------|-----------------------------------|-------------|------------------
Documentation | [DESIGN_DELTAS_CAPTURED_V3][]     | N/A         |
Tests         | [X_PROP_ANALYSIS_COMPLETED][]     | Waived      | Revisit later. Tool setup in progress.
Tests         | [FPV_ASSERTIONS_PROVEN_AT_V3][]   | N/A         |
Regression    | [SIM_NIGHTLY_REGRESSION_AT_V3][]  | Done        | Resolved: [#680][]
Coverage      | [SIM_CODE_COVERAGE_AT_100][]      | Done        | [common_cov_excl.el][], [gpio_cov_excl.el][]
Coverage      | [SIM_FUNCTIONAL_COVERAGE_AT_100][]| Done        | [#807][]
Coverage      | [FPV_CODE_COVERAGE_AT_100][]      | N/A         |
Coverage      | [FPV_COI_COVERAGE_AT_100][]       | N/A         |
Code Quality  | [ALL_TODOS_RESOLVED][]            | Done        |
Code Quality  | [NO_TOOL_WARNINGS_THROWN][]       | Done        |
Code Quality  | [TB_LINT_COMPLETE][]              | Not Started |
Integration   | [PRE_VERIFIED_SUB_MODULES_V3][]   | N/A         |
Issues        | [NO_ISSUES_PENDING][]             | Done        |
Review        | Reviewer(s)                       | Done        | @eunchan @sriyerg @sjgitty
Review        | Signoff date                      | Done        | 2019-11-04

[#807]: https://github.com/lowRISC/opentitan/pull/807

[DESIGN_DELTAS_CAPTURED_V3]:     ../../../../../doc/project_governance/checklist/README.md#design_deltas_captured_v3
[X_PROP_ANALYSIS_COMPLETED]:     ../../../../../doc/project_governance/checklist/README.md#x_prop_analysis_completed
[FPV_ASSERTIONS_PROVEN_AT_V3]:   ../../../../../doc/project_governance/checklist/README.md#fpv_assertions_proven_at_v3
[SIM_NIGHTLY_REGRESSION_AT_V3]:  ../../../../../doc/project_governance/checklist/README.md#sim_nightly_regression_at_v3
[SIM_CODE_COVERAGE_AT_100]:      ../../../../../doc/project_governance/checklist/README.md#sim_code_coverage_at_100
[SIM_FUNCTIONAL_COVERAGE_AT_100]:../../../../../doc/project_governance/checklist/README.md#sim_functional_coverage_at_100
[FPV_CODE_COVERAGE_AT_100]:      ../../../../../doc/project_governance/checklist/README.md#fpv_code_coverage_at_100
[FPV_COI_COVERAGE_AT_100]:       ../../../../../doc/project_governance/checklist/README.md#fpv_coi_coverage_at_100
[ALL_TODOS_RESOLVED]:            ../../../../../doc/project_governance/checklist/README.md#all_todos_resolved
[NO_TOOL_WARNINGS_THROWN]:       ../../../../../doc/project_governance/checklist/README.md#no_tool_warnings_thrown
[TB_LINT_COMPLETE]:              ../../../../../doc/project_governance/checklist/README.md#tb_lint_complete
[PRE_VERIFIED_SUB_MODULES_V3]:   ../../../../../doc/project_governance/checklist/README.md#pre_verified_sub_modules_v3
[NO_ISSUES_PENDING]:             ../../../../../doc/project_governance/checklist/README.md#no_issues_pending

[common_cov_excl.el]:https://github.com/lowRISC/opentitan/blob/9dff09b6c57f4962d67f5f64f8e69ac9bea6885c/hw/dv/tools/vcs/common_cov_excl.el
[gpio_cov_excl.el]:  https://github.com/lowRISC/opentitan/blob/39aaeefdb43661b065c29ceab2efc1065aebf6dd/hw/ip/gpio/dv/cov/gpio_cov_excl.el
