# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

package(default_visibility = ["//visibility:public"])

load(
    "//rules:autogen.bzl",
    "autogen_hjson_c_header",
    "autogen_hjson_rust_header",
)

autogen_hjson_c_header(
    name = "dma_c_regs",
    srcs = [
        "dma.hjson",
    ],
)

autogen_hjson_rust_header(
    name = "dma_rust_regs",
    srcs = [
        "dma.hjson",
    ],
)

filegroup(
    name = "all_files",
    srcs = glob(["**"]),
)
