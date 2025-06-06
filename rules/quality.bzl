# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

"""Quality check rules for OpenTitan.
"""

load("@rules_cc//cc:action_names.bzl", "ACTION_NAMES", "C_COMPILE_ACTION_NAME")
load("@bazel_skylib//lib:shell.bzl", "shell")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain")
load("//rules:rv.bzl", "rv_rule")
load("//rules/opentitan:toolchain.bzl", "LOCALTOOLS_TOOLCHAIN")

def _ensure_tag(tags, *tag):
    for t in tag:
        if t not in tags:
            tags.append(t)
    return tags

def _cc_aspect_impl(target, ctx, action_callback):
    """Aspect implementation for C/C++ targets with configurable callback."""

    def make_output_group_info(files = []):
        """Create an OutputGroupInfo with a name defined by `ctx`."""
        args = {ctx.attr._output_group_name: files}
        return OutputGroupInfo(**args)

    if ctx.rule.kind not in ["cc_library", "cc_binary", "cc_test"]:
        return [make_output_group_info()]

    if CcInfo in target:
        cc_info = target[CcInfo]
    elif hasattr(ctx.rule.attr, "deps"):
        # Some rules, like cc_binary, do NOT produce a CcInfo provider. Therefore,
        # we need to build one from its dependencies.
        cc_info = cc_common.merge_cc_infos(
            direct_cc_infos = [dep[CcInfo] for dep in ctx.rule.attr.deps if CcInfo in dep],
        )
    else:
        return [make_output_group_info()]
    cc_compile_ctx = cc_info.compilation_context

    cc_toolchain = find_cc_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    if not hasattr(ctx.rule.attr, "srcs"):
        return [make_output_group_info()]
    all_srcs = []
    for src in ctx.rule.attr.srcs:
        all_srcs += [src for src in src.files.to_list() if src.is_source]

    outputs = []
    for src in all_srcs:
        if src.path.startswith("external/"):
            continue
        if not src.extension in ["c", "cc", "h"]:
            continue

        generated_file = ctx.actions.declare_file(
            "{}.{}.{}".format(src.basename, target.label.name, ctx.attr._output_file_suffix),
        )
        outputs.append(generated_file)

        opts = ctx.fragments.cpp.copts
        if hasattr(ctx.rule.attr, "copts"):
            opts += ctx.rule.attr.copts

        # TODO(dmcardle) What if an .h file should be compiled for C++?
        if src.extension in ["c", "h"]:
            opts += ctx.fragments.cpp.conlyopts
        else:
            opts += ctx.fragments.cpp.cxxopts
            if hasattr(ctx.rule.attr, "cxxopts"):
                opts += ctx.rule.attr.cxxopts

        c_compile_variables = cc_common.create_compile_variables(
            feature_configuration = feature_configuration,
            cc_toolchain = cc_toolchain,
            source_file = src.path,
            user_compile_flags = opts,
            include_directories = depset(
                direct = [src.dirname for src in cc_compile_ctx.headers.to_list()],
                transitive = [cc_compile_ctx.includes],
            ),
            quote_include_directories = cc_compile_ctx.quote_includes,
            system_include_directories = cc_compile_ctx.system_includes,
            framework_include_directories = cc_compile_ctx.framework_includes,
            preprocessor_defines = depset(
                direct = ctx.rule.attr.local_defines,
                transitive = [cc_compile_ctx.defines],
            ),
        )

        command_line = cc_common.get_memory_inefficient_command_line(
            feature_configuration = feature_configuration,
            action_name = ACTION_NAMES.c_compile,
            variables = c_compile_variables,
        )

        action_callback(ctx, cc_toolchain, cc_compile_ctx, generated_file, command_line, src)

    return [
        make_output_group_info(depset(direct = outputs)),
    ]

# To see which checks clang-tidy knows about, run this command:
#
#  ./bazelisk.sh run @lowrisc_rv32imcb_toolchain//:bin/clang-tidy -- --checks='*' --list-checks
_CLANG_TIDY_CHECKS = [
    "clang-analyzer-core.*",
    # Disable advice to replace `memcpy` with `mempcy_s`.
    "-clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling",
]

def _clang_tidy_run_action(ctx, cc_toolchain, cc_compile_ctx, generated_file, command_line, src):
    """Generates an action to run clang-tidy."""

    args = ctx.actions.args()

    # Add args that are consumed by the wrapper script.
    if ctx.attr._enable_fix:
        args.add("--ignore-clang-tidy-error")
    args.add(".clang-tidy.lock")
    args.add(generated_file)
    args.add_all(ctx.attr._clang_tidy.files)

    # Add args for clang-tidy.
    if len(_CLANG_TIDY_CHECKS) > 0:
        checks_pattern = ",".join(_CLANG_TIDY_CHECKS)
        args.add("--checks=" + checks_pattern)

        # Treat warnings from every enabled check as errors.
        args.add("--warnings-as-errors=" + checks_pattern)
    if ctx.attr._enable_fix:
        args.add("--fix")
        args.add("--fix-errors")

        # Use the nearest .clang_format file to format code adjacent to fixes.
        args.add("--format-style=file")

    # Specify a regex header filter. Without this, clang-tidy will not
    # report or fix errors in header files.
    args.add("--header-filter=.*\\.h$")
    args.add("--use-color")

    for arg in command_line:
        if arg in [src.path, "-fno-canonical-system-headers"]:
            continue
        args.add("--extra-arg=" + arg)

    # Tell clang-tidy which source file to analyze.
    args.add(src)

    ctx.actions.run(
        executable = ctx.attr._clang_tidy_wrapper.files_to_run,
        arguments = [args],
        inputs = depset(
            direct = [src],
            transitive = [
                cc_toolchain.all_files,
                cc_compile_ctx.headers,
            ],
        ),
        tools = [ctx.attr._clang_tidy.files_to_run],
        outputs = [generated_file],
        progress_message = "Running clang tidy on {}".format(src.path),
    )

def _make_clang_tidy_aspect(enable_fix):
    return aspect(
        implementation = lambda target, ctx: _cc_aspect_impl(target, ctx, _clang_tidy_run_action),
        attr_aspects = ["deps"],
        attrs = {
            "_clang_tidy_wrapper": attr.label(
                default = "//rules/scripts:clang_tidy.py",
                allow_single_file = True,
                cfg = "host",
                executable = True,
            ),
            "_clang_tidy": attr.label(
                default = "@lowrisc_rv32imcb_toolchain//:bin/clang-tidy",
                allow_single_file = True,
                cfg = "host",
                executable = True,
                doc = "The clang-tidy executable",
            ),
            "_enable_fix": attr.bool(default = enable_fix),
            "_output_group_name": attr.string(default = "clang_tidy"),
            "_output_file_suffix": attr.string(default = "clang-tidy.out"),
        },
        incompatible_use_toolchain_transition = True,
        fragments = ["cpp"],
        host_fragments = ["cpp"],
        toolchains = ["@rules_cc//cc:toolchain_type"],
    )

clang_tidy_fix_aspect = _make_clang_tidy_aspect(True)
clang_tidy_check_aspect = _make_clang_tidy_aspect(False)

def _audit_sec_mmio_calls_run_action(ctx, cc_toolchain, cc_compile_ctx, generated_file, command_line, src):
    """Generates an action that runs the sec_mmio audit script on one source."""

    args = ctx.actions.args()
    args.add(generated_file)
    args.add(src)

    for arg in command_line:
        if arg in [src.path, "-fno-canonical-system-headers", "-c"]:
            continue
        elif arg.startswith("-march=rv32imc"):
            continue
        args.add(arg)

    ctx.actions.run(
        executable = ctx.attr._audit_tool.files_to_run,
        arguments = [args],
        inputs = depset(
            direct = [src],
            transitive = [
                cc_toolchain.all_files,
                cc_compile_ctx.headers,
            ],
        ),
        outputs = [generated_file],
    )

audit_sec_mmio_calls_aspect = aspect(
    implementation = lambda target, ctx: _cc_aspect_impl(target, ctx, _audit_sec_mmio_calls_run_action),
    attr_aspects = ["deps"],
    attrs = {
        "_audit_tool": attr.label(
            default = "//util/py/scripts:bazel_aspect_tool_audit_sec_mmio_calls",
            cfg = "host",
            executable = True,
        ),
        "_output_group_name": attr.string(default = "audit_sec_mmio"),
        "_output_file_suffix": attr.string(default = "clang-audit.out"),
    },
    incompatible_use_toolchain_transition = True,
    fragments = ["cpp"],
    host_fragments = ["cpp"],
    toolchains = ["@rules_cc//cc:toolchain_type"],
)

def _clang_tidy_test_impl(ctx):
    # Test rules must produce an exectuable, so create a dummy script. If the
    # clang-tidy rules were not test rules, the targets they instantiate could
    # not depend on test targets. For context, see issue #18726.
    out_file = ctx.actions.declare_file(ctx.label.name + ".dummy.bash")
    ctx.actions.write(out_file, "", is_executable = True)

    return [
        DefaultInfo(
            files = depset(
                transitive = [dep[OutputGroupInfo].clang_tidy for dep in ctx.attr.deps],
            ),
            executable = out_file,
        ),
    ]

clang_tidy_rv_test = rv_rule(
    implementation = _clang_tidy_test_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [clang_tidy_check_aspect],
        ),
    },
    test = True,
)

clang_tidy_test = rule(
    implementation = _clang_tidy_test_impl,
    attrs = {
        "deps": attr.label_list(
            aspects = [clang_tidy_check_aspect],
        ),
    },
    test = True,
)

def _html_coverage_report_impl(ctx):
    out_file = ctx.actions.declare_file(ctx.label.name + ".bash")
    substitutions = {}
    ctx.actions.expand_template(
        template = ctx.file._runner,
        output = out_file,
        substitutions = substitutions,
        is_executable = True,
    )

    return DefaultInfo(
        files = depset([out_file]),
        executable = out_file,
    )

html_coverage_report = rule(
    implementation = _html_coverage_report_impl,
    attrs = {
        "_runner": attr.label(
            default = "//rules/scripts:html_coverage_report.template.sh",
            allow_single_file = True,
        ),
    },
    executable = True,
)

HasModuleIdInfo = provider()

def _modid_check_aspect_impl(target, ctx):
    """
    Verify that a binary (ELF file) does not contain conflicting module IDs
    using opentitantool.
    """
    tc = ctx.toolchains[LOCALTOOLS_TOOLCHAIN]

    # If the target is //sw/device/lib/base:status, then it has module ID information,
    # this is the root of all the information.
    if ctx.label == Label("@//sw/device/lib/base:status"):
        return [HasModuleIdInfo()]

    # Ignore everything outside of //sw/device/
    if not ctx.label.package.startswith("sw/device"):
        return []

    # If any of the dependencies has module ID info, then this target will have it
    has_modid_info = hasattr(ctx.rule.attr, "deps") and any([HasModuleIdInfo in dep for dep in ctx.rule.attr.deps])

    if not has_modid_info:
        return []

    # If it is not a binary, propagate up.
    if ctx.rule.kind != "cc_binary":
        return [HasModuleIdInfo()]

    # We create a file that will not contain anything: this is just to create a "link"
    # between the run action and the output group info. This way if we ask bazel for this
    # output group, it will automatically run the action. We could use a validation group
    # but at least this makes the check more explicit.
    generated_file = ctx.actions.declare_file("{}.mod-id".format(target.label.name))

    # Call bash script that will run opentitantool and capture the output. We want to avoid
    # printing anything if the test is successful but by default opentitantool prints
    # unnecessary information that pollutes the output.
    args = ctx.actions.args()

    #  The opentitantool binary returns a FilesToRun provider.
    args.add_all([tc.tools.opentitantool.executable.path, generated_file])
    args.add_all(target.files)
    ctx.actions.run(
        executable = ctx.executable._modid_check,
        arguments = [args],
        inputs = target.files,
        tools = [tc.tools.opentitantool],
        outputs = [generated_file],
        progress_message = "Checking module IDs for %{label}",
    )

    return [
        OutputGroupInfo(
            modid_checks = depset(direct = [generated_file]),
        ),
    ]

modid_check_aspect = aspect(
    implementation = _modid_check_aspect_impl,
    # Propagate down all dependencies so we go through test suites and other
    # types of dependencies to reach the binaries.
    attr_aspects = ["*"],
    attrs = {
        "_modid_check": attr.label(
            default = "//rules/scripts:modid_check",
            executable = True,
            cfg = "exec",
        ),
    },
    toolchains = [LOCALTOOLS_TOOLCHAIN],
)
