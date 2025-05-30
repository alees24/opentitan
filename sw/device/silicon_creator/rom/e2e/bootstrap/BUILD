# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

load(
    "//rules/opentitan:defs.bzl",
    "cw310_params",
    "ecdsa_key_for_lc_state",
    "fpga_params",
    "opentitan_test",
)
load("//rules/opentitan:keyutils.bzl", "ECDSA_ONLY_KEY_STRUCTS")
load("//rules:const.bzl", "CONST")
load(
    "//rules:otp.bzl",
    "STD_OTP_OVERLAYS",
    "otp_hex",
    "otp_image",
    "otp_json",
    "otp_partition",
)

package(default_visibility = ["//visibility:public"])

otp_json(
    name = "otp_json_bootstrap_rma",
    partitions = [
        otp_partition(
            name = "CREATOR_SW_CFG",
            items = {
                "CREATOR_SW_CFG_RMA_SPIN_EN": otp_hex(CONST.HARDENED_TRUE),
                # Number of Ibex cycles to spin: approximately 5.5s @ 24Mhz
                # If you change this value, make sure to update the code in the harness.
                "CREATOR_SW_CFG_RMA_SPIN_CYCLES": "0x8000000",
            },
        ),
        otp_partition(
            name = "SECRET2",
            items = {
                # This RMA token is a cSHAKE128 digest. The preimage is
                # hardcoded into the test harness [0] and the tool that
                # generated this token [1].
                #
                # [0]: //sw/host/tests/rom/e2e_bootstrap_rma
                # [1]: //sw/host/tests/rom/e2e_bootstrap_rma:gen_rma_token
                "RMA_TOKEN": "0x1faf9056acde66561685549803a28bec",
                "CREATOR_ROOT_KEY_SHARE0": "<random>",
                "CREATOR_ROOT_KEY_SHARE1": "<random>",
            },
            lock = True,
        ),
    ],
    visibility = ["//visibility:private"],
)

# This OTP image is tightly coupled with the "rom_bootstrap_rma" testpoint.
# Despite its name ending with "rma", it actually puts the device into the PROD
# lifecycle state in order to test the transition from PROD to RMA.
otp_image(
    name = "otp_img_bootstrap_rma",
    src = "//hw/top_earlgrey/data/otp:otp_json_prod",
    overlays = STD_OTP_OVERLAYS + [
        ":otp_json_bootstrap_rma",
    ],
    visibility = ["//visibility:private"],
)

opentitan_test(
    name = "e2e_bootstrap_rma",
    srcs = ["rom_e2e_bootstrap_rma_test.c"],
    ecdsa_key = ecdsa_key_for_lc_state(
        ECDSA_ONLY_KEY_STRUCTS,
        CONST.LCV.PROD,
    ),
    exec_env = {
        "//hw/top_earlgrey:fpga_cw310_rom_with_fake_keys": None,
    },
    fpga = fpga_params(
        changes_otp = True,
        needs_jtag = True,
        otp = "otp_img_bootstrap_rma",
        test_cmd = """
            --bootstrap="{firmware}"
        """,
        test_harness = "//sw/host/tests/rom/e2e_bootstrap_rma",
    ),
    deps = [
        "//hw/top_earlgrey/sw/autogen:top_earlgrey",
        "//sw/device/lib/base:macros",
        "//sw/device/lib/runtime:log",
        "//sw/device/lib/testing/test_framework:ottf_main",
        "//sw/device/silicon_creator/lib/drivers:retention_sram",
    ],
)

opentitan_test(
    name = "e2e_bootstrap_disabled",
    exec_env = {
        "//hw/top_earlgrey:fpga_cw310_rom_with_fake_keys": None,
    },
    fpga = fpga_params(
        # Since the bitstream disables bootstrap, there is no firmware to
        # load into the chip.  However, opentitan_test wants to build a
        # binary target.  We'll build an unsigned do-nothing binary.
        binaries = {
            "//sw/device/silicon_creator/rom/e2e:new_empty_test_slot_a": "firmware",
        },
        otp = "//hw/top_earlgrey/data/otp:img_bootstrap_disabled",
        test_harness = "//sw/host/tests/rom/e2e_bootstrap_disabled",
    ),
    manifest = None,
)
