# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

load("@rules_python//python:defs.bzl", "py_test")
load("@ot_python_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

exports_files(glob(["**"]))

py_library(
    name = "bitstreams_workspace",
    srcs = [
        "bitstreams_workspace.py",
    ],
    deps = [
        requirement("jsonschema"),
    ],
)

filegroup(
    name = "bitstreams_manifest_schema",
    srcs = ["bitstreams_manifest.schema.json"],
)

py_test(
    name = "bitstreams_workspace_test",
    srcs = [
        "bitstreams_workspace_test.py",
    ],
    deps = [
        ":bitstreams_workspace",
    ],
)

sh_binary(
    name = "modid_check",
    srcs = ["modid_check.sh"],
)
