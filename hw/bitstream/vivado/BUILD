# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

load("@rules_pkg//pkg:mappings.bzl", "pkg_filegroup", "pkg_files")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("//rules:const.bzl", "KEY_AUTHENTICITY")
load("//rules:fusesoc.bzl", "fusesoc_build")
load("//rules:otp.bzl", "get_otp_images")
load("//rules:bitstreams.bzl", "bitstream_manifest_fragment")

package(default_visibility = ["//visibility:public"])

# The readmem directives in the fusesoc-ized build tree will be in the subdir
# ${build_root}/${core}/${target}-${tool}/src/lowrisc_prim_util_memload_0/rtl/prim_util_memload.svh,
# and ${build_root} will be a subdirectory called `build.fpga_cw310` inside of
# bazel-out/k8-{configname}/bin/hw/bitstream/vivado.
# Therefore, the relative path between prim_util_memload.svh and the project-root
# relative $(location ...) resolved labels is up 10 subdirectories.
_PREFIX = "../../../../../../../../../../.."

_CW310_TESTROM = "//sw/device/lib/testing/test_rom:test_rom_fpga_cw310_scr_vmem"

_CW340_TESTROM = _CW310_TESTROM

_OTP_RMA = "//hw/top_earlgrey/data/otp:img_rma"

_CW310_TESTROM_PATH = "{}/$(location {})".format(_PREFIX, _CW310_TESTROM)

_CW340_TESTROM_PATH = _CW310_TESTROM_PATH

_OTP_RMA_PATH = "{}/$(location {})".format(_PREFIX, _OTP_RMA)

_FPGA_PATH_TMPL = "lowrisc_systems_{}_0.1/synth-vivado/{}"

# Note: all of the targets are tagged with "manual" to prevent them from being
# matched by bazel wildcards like "//...".  In order to build the bitstream,
# you need to ask for it directly or by dependency via another rule, such as
# a functest.
fusesoc_build(
    name = "fpga_cw310",
    testonly = True,
    srcs = [
        "//hw:rtl_files",
        _CW310_TESTROM,
        _OTP_RMA,
    ],
    cores = ["//hw:cores"],
    data = ["//hw/ip/otbn:rtl_files"],
    flags = [
        "--BootRomInitFile=" + _CW310_TESTROM_PATH,
        "--OtpMacroMemInitFile=" + _OTP_RMA_PATH,
    ],
    output_groups = {
        "bitstream": [_FPGA_PATH_TMPL.format("chip_earlgrey_cw310", "lowrisc_systems_chip_earlgrey_cw310_0.1.bit")],
        "mmi": [_FPGA_PATH_TMPL.format("chip_earlgrey_cw310", "memories.mmi")],
        "logs": [_FPGA_PATH_TMPL.format("chip_earlgrey_cw310", "lowrisc_systems_chip_earlgrey_cw310_0.1.runs/")],
    },
    systems = ["lowrisc:systems:chip_earlgrey_cw310"],
    tags = ["manual"],
    target = "synth",
)

filegroup(
    name = "fpga_cw310_test_rom",
    testonly = True,
    srcs = [":fpga_cw310"],
    output_group = "bitstream",
    tags = ["manual"],
)

filegroup(
    name = "cw310_mmi",
    testonly = True,
    srcs = [":fpga_cw310"],
    output_group = "mmi",
    tags = ["manual"],
)

# Standalone CW310 image for use with hyperdebug.
fusesoc_build(
    name = "fpga_cw310_hyperdebug",
    testonly = True,
    srcs = [
        "//hw:rtl_files",
        _CW310_TESTROM,
        _OTP_RMA,
    ],
    cores = ["//hw:cores"],
    data = ["//hw/ip/otbn:rtl_files"],
    flags = [
        "--BootRomInitFile=" + _CW310_TESTROM_PATH,
        "--OtpMacroMemInitFile=" + _OTP_RMA_PATH,
    ],
    output_groups = {
        "bitstream": [_FPGA_PATH_TMPL.format("chip_earlgrey_cw310_hyperdebug", "lowrisc_systems_chip_earlgrey_cw310_hyperdebug_0.1.bit")],
        "mmi": [_FPGA_PATH_TMPL.format("chip_earlgrey_cw310_hyperdebug", "memories.mmi")],
        "logs": [_FPGA_PATH_TMPL.format("chip_earlgrey_cw310_hyperdebug", "lowrisc_systems_chip_earlgrey_cw310_hyperdebug_0.1.runs/")],
    },
    systems = ["lowrisc:systems:chip_earlgrey_cw310_hyperdebug"],
    tags = ["manual"],
    target = "synth",
)

filegroup(
    name = "fpga_cw310_test_rom_hyp",
    testonly = True,
    srcs = [":fpga_cw310_hyperdebug"],
    output_group = "bitstream",
    tags = ["manual"],
)

filegroup(
    name = "cw310_hyperdebug_mmi",
    testonly = True,
    srcs = [":fpga_cw310_hyperdebug"],
    output_group = "mmi",
    tags = ["manual"],
)

# CW340 bitstream
fusesoc_build(
    name = "fpga_cw340",
    testonly = True,
    srcs = [
        "//hw:rtl_files",
        _CW340_TESTROM,
        _OTP_RMA,
    ],
    cores = ["//hw:cores"],
    data = ["//hw/ip/otbn:rtl_files"],
    flags = [
        "--BootRomInitFile=" + _CW340_TESTROM_PATH,
        "--OtpMacroMemInitFile=" + _OTP_RMA_PATH,
    ],
    output_groups = {
        "bitstream": [_FPGA_PATH_TMPL.format("chip_earlgrey_cw340", "lowrisc_systems_chip_earlgrey_cw340_0.1.bit")],
        "mmi": [_FPGA_PATH_TMPL.format("chip_earlgrey_cw340", "memories.mmi")],
        "logs": [_FPGA_PATH_TMPL.format("chip_earlgrey_cw340", "lowrisc_systems_chip_earlgrey_cw340_0.1.runs/")],
    },
    systems = ["lowrisc:systems:chip_earlgrey_cw340"],
    tags = ["manual"],
    target = "synth",
)

filegroup(
    name = "fpga_cw340_test_rom",
    testonly = True,
    srcs = [":fpga_cw340"],
    output_group = "bitstream",
    tags = ["manual"],
)

filegroup(
    name = "cw340_mmi",
    testonly = True,
    srcs = [":fpga_cw340"],
    output_group = "mmi",
    tags = ["manual"],
)

# Bitstream cache manifest fragment targets
bitstream_manifest_fragment(
    name = "earlgrey_cw310_manifest",
    testonly = True,
    srcs = [":fpga_cw310"],
    bitstream = "bitstream",
    design = "chip_earlgrey_cw310",
    memories = [
        "rom",
        "otp",
        "flash0_info0",
        "flash0_info1",
        "flash0_info2",
        "flash1_info0",
        "flash1_info1",
        "flash1_info2",
    ],
    memory_map_file = ":cw310_mmi",
    tags = ["manual"],
)

pkg_tar(
    name = "earlgrey_cw310_archive",
    testonly = True,
    srcs = [":earlgrey_cw310_manifest"],
    mode = "0444",
    package_dir = "build-bin/hw/top_earlgrey/chip_earlgrey_cw310",
    strip_prefix = "/hw/bitstream/vivado/earlgrey_cw310_manifest",
    tags = ["manual"],
)

bitstream_manifest_fragment(
    name = "earlgrey_cw310_hyperdebug_manifest",
    testonly = True,
    srcs = [":fpga_cw310_hyperdebug"],
    bitstream = "bitstream",
    design = "chip_earlgrey_cw310_hyperdebug",
    memories = [
        "rom",
        "otp",
        "flash0_info0",
        "flash0_info1",
        "flash0_info2",
        "flash1_info0",
        "flash1_info1",
        "flash1_info2",
    ],
    memory_map_file = ":cw310_hyperdebug_mmi",
    tags = ["manual"],
)

pkg_tar(
    name = "earlgrey_cw310_hyperdebug_archive",
    testonly = True,
    srcs = [":earlgrey_cw310_hyperdebug_manifest"],
    mode = "0444",
    package_dir = "build-bin/hw/top_earlgrey/chip_earlgrey_cw310_hyperdebug",
    strip_prefix = "/hw/bitstream/vivado/earlgrey_cw310_hyperdebug_manifest",
    tags = ["manual"],
)

bitstream_manifest_fragment(
    name = "earlgrey_cw340_manifest",
    testonly = True,
    srcs = [":fpga_cw340"],
    bitstream = "bitstream",
    design = "chip_earlgrey_cw340",
    memories = [
        "rom",
        "otp",
        "flash0_info0",
        "flash0_info1",
        "flash0_info2",
        "flash1_info0",
        "flash1_info1",
        "flash1_info2",
    ],
    memory_map_file = ":cw340_mmi",
    tags = ["manual"],
)

pkg_tar(
    name = "earlgrey_cw340_archive",
    testonly = True,
    srcs = [":earlgrey_cw340_manifest"],
    mode = "0444",
    package_dir = "build-bin/hw/top_earlgrey/chip_earlgrey_cw340",
    strip_prefix = "/hw/bitstream/vivado/earlgrey_cw340_manifest",
    tags = ["manual"],
)

# Packaging rules for bitstreams
pkg_files(
    name = "standard",
    testonly = True,
    srcs = [
        ":cw310_mmi",
        ":fpga_cw310_test_rom",
    ],
    prefix = "earlgrey/fpga_cw310/standard",
    tags = ["manual"],
)

pkg_files(
    name = "hyperdebug",
    testonly = True,
    srcs = [
        ":cw310_hyperdebug_mmi",
        ":fpga_cw310_test_rom_hyp",
    ],
    prefix = "earlgrey/fpga_cw310/hyperdebug",
    tags = ["manual"],
)

pkg_filegroup(
    name = "package",
    testonly = True,
    srcs = [
        ":hyperdebug",
        ":standard",
    ],
    tags = ["manual"],
)
