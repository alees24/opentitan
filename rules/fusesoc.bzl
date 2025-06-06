# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

load("@bazel_skylib//lib:dicts.bzl", "dicts")
load("@nonhermetic//:env.bzl", "BIN_PATHS", "ENV")

"""Rules for running FuseSoC.

FuseSoC is a package manager and set of build tools for HDL code.

Because we want the output of some FuseSoC built resources to be
available to bazel (such as the verilated chip model for running
tests), the `fusesoc_build` rule allows bazel to delegate certain
targets to FuseSoC.

This rule is not sandboxed, as our current configuration depends
on FuseSoC and its dependencies (verible, verilator, etc) already
having been installed.  In the future, we will try to rework our
dependencies so the FuseSoC rules can be sandboxed.
"""

load("@bazel_skylib//rules:common_settings.bzl", "BuildSettingInfo")

def _corefiles2rootarg(core):
    return core.dirname

def _fusesoc_build_impl(ctx):
    build_dir = "build.{}".format(ctx.label.name)
    out_dir = "{}/{}/{}".format(ctx.bin_dir.path, ctx.label.package, build_dir)
    flags = [ctx.expand_location(f, ctx.attr.srcs) for f in ctx.attr.flags]
    outputs = []
    groups = {}

    # Vivado expects `HOME` environment variable to exist. Redirect it to a fake directory.
    home_dir = "{}/homeless-shelter".format(out_dir)

    # TODO(#27346): Use of `/tmp` here isn't hermetic.
    cache_dir = "/tmp/fusesoc-cache"
    cfg_file_path = "build.{}.fusesoc_config.toml".format(ctx.label.name)
    cfg_file = ctx.actions.declare_file(cfg_file_path)
    cfg_str = "[main]\n  cache_root = {}".format(cache_dir)
    ctx.actions.write(cfg_file, cfg_str)

    args = ctx.actions.args()
    args.add(cfg_file.path, format = "--config=%s")

    for group, files in ctx.attr.output_groups.items():
        deps = []
        for file in files:
            path = "{}/{}".format(build_dir, file)
            if file.endswith("/"):
                deps.append(ctx.actions.declare_directory(path))
            else:
                deps.append(ctx.actions.declare_file(path))
        outputs.extend(deps)
        groups[group] = depset(deps)

    if ctx.attr.verilator_options:
        verilator_options = ctx.attr.verilator_options[BuildSettingInfo].value
        flags.append("--verilator_options={}".format(" ".join(verilator_options)))

    if ctx.attr.make_options:
        make_options = ctx.attr.make_options[BuildSettingInfo].value
        flags.append("--make_options={}".format(" ".join(make_options)))

    args.add_all(
        ctx.files.cores,
        uniquify = True,
        map_each = _corefiles2rootarg,
        format_each = "--cores-root=%s",
    )

    args.add("run")
    args.add(ctx.attr.target, format = "--target=%s")
    args.add_all([
        "--setup",
        "--build",
    ])
    args.add(out_dir, format = "--build-root=%s")

    args.add_all(ctx.attr.systems)
    args.add_all(flags)

    ctx.actions.run(
        mnemonic = "FuseSoC",
        outputs = outputs,
        inputs = ctx.files.srcs + ctx.files.cores + ctx.files._fusesoc + [
            cfg_file,
        ],
        arguments = [args],
        executable = ctx.executable._fusesoc,
        use_default_shell_env = False,
        env = dicts.add(
            # Verilator build doesn't need nonhermetic environment variables
            ENV if ctx.attr.target == "synth" else {},
            {
                "HOME": home_dir,
                # Obtain the non-hermetic binary path and append Bazel's default PATH.
                "PATH": BIN_PATHS["vivado" if ctx.attr.target == "synth" else "verilator"] + ":/bin:/usr/bin:/usr/local/bin",
            },
        ),
    )
    return [
        DefaultInfo(
            files = depset(outputs),
            data_runfiles = ctx.runfiles(files = outputs + ctx.files.data),
        ),
        OutputGroupInfo(**groups),
    ]

fusesoc_build = rule(
    implementation = _fusesoc_build_impl,
    attrs = {
        "cores": attr.label_list(allow_files = True, doc = "FuseSoC core specification files"),
        "srcs": attr.label_list(allow_files = True, doc = "Source files"),
        "data": attr.label_list(allow_files = True, doc = "Files needed at runtime"),
        "target": attr.string(mandatory = True, doc = "Target name (e.g. 'sim')"),
        "systems": attr.string_list(mandatory = True, doc = "Systems to build"),
        "flags": attr.string_list(doc = "Flags controlling the FuseSOC system build"),
        "output_groups": attr.string_list_dict(
            allow_empty = True,
            doc = """
                Mappings from output group names to lists of paths contained in
                that group.

                Paths to directories must have a trailing `/`. It is not
                possible to output both a directory and a file from within that
                directory.
            """,
        ),
        "verilator_options": attr.label(),
        "make_options": attr.label(),
        "_fusesoc": attr.label(
            default = "//util:fusesoc_build",
            executable = True,
            cfg = "exec",
        ),
    },
)
