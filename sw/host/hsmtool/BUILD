# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

load("@rules_rust//rust:defs.bzl", "rust_binary", "rust_doc", "rust_library", "rust_test")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "doc_files",
    srcs = glob(["**/*.md"]),
)

alias(
    name = "binding_srcs",
    actual = "@crate_index//:cryptoki-sys-binding-srcs",
)

py_binary(
    name = "pkcs11_consts",
    srcs = ["scripts/pkcs11_consts.py"],
)

genrule(
    name = "object_class",
    srcs = [":binding_srcs"],
    outs = ["object_class.rs"],
    cmd = """
        $(execpath :pkcs11_consts) \
            --binding $(locations :binding_srcs) \
            --serde --strum --conv_data CKO cryptoki::object::ObjectClass > $@
    """,
    tools = [
        ":pkcs11_consts",
    ],
)

genrule(
    name = "key_type",
    srcs = [":binding_srcs"],
    outs = ["key_type.rs"],
    cmd = """
        $(execpath :pkcs11_consts) \
            --binding $(locations :binding_srcs) \
            --serde --strum --conv_data CKK cryptoki::object::KeyType > $@
    """,
    tools = [
        ":pkcs11_consts",
    ],
)

genrule(
    name = "certificate_type",
    srcs = [":binding_srcs"],
    outs = ["certificate_type.rs"],
    cmd = """
        $(execpath :pkcs11_consts) \
            --binding $(locations :binding_srcs) \
            --serde --strum --conv_data CKC cryptoki::object::CertificateType > $@
    """,
    tools = [
        ":pkcs11_consts",
    ],
)

genrule(
    name = "mechanism_type",
    srcs = [":binding_srcs"],
    outs = ["mechanism_type.rs"],
    cmd = """
        $(execpath :pkcs11_consts) \
            --binding $(locations :binding_srcs) \
            --serde --strum --conv_data CKM cryptoki::mechanism::MechanismType > $@
    """,
    tools = [
        ":pkcs11_consts",
    ],
)

genrule(
    name = "attribute_type",
    srcs = [":binding_srcs"],
    outs = ["attribute_type.rs"],
    cmd = """
        $(execpath :pkcs11_consts) \
            --binding $(locations :binding_srcs) \
            --serde --strum CKA cryptoki::object::AttributeType > $@
    """,
    tools = [
        ":pkcs11_consts",
    ],
)

rust_library(
    name = "hsmlib",
    srcs = [
        "src/commands/ecdsa/export.rs",
        "src/commands/ecdsa/generate.rs",
        "src/commands/ecdsa/import.rs",
        "src/commands/ecdsa/mod.rs",
        "src/commands/ecdsa/sign.rs",
        "src/commands/ecdsa/verify.rs",
        "src/commands/exec.rs",
        "src/commands/mod.rs",
        "src/commands/object/destroy.rs",
        "src/commands/object/list.rs",
        "src/commands/object/mod.rs",
        "src/commands/object/read.rs",
        "src/commands/object/show.rs",
        "src/commands/object/update.rs",
        "src/commands/object/write.rs",
        "src/commands/rsa/decrypt.rs",
        "src/commands/rsa/encrypt.rs",
        "src/commands/rsa/export.rs",
        "src/commands/rsa/generate.rs",
        "src/commands/rsa/import.rs",
        "src/commands/rsa/mod.rs",
        "src/commands/rsa/sign.rs",
        "src/commands/rsa/verify.rs",
        "src/commands/spx/export.rs",
        "src/commands/spx/generate.rs",
        "src/commands/spx/import.rs",
        "src/commands/spx/list.rs",
        "src/commands/spx/mod.rs",
        "src/commands/spx/sign.rs",
        "src/commands/spx/verify.rs",
        "src/commands/token.rs",
        "src/error.rs",
        "src/lib.rs",
        "src/module.rs",
        "src/profile.rs",
        "src/spxef/mod.rs",
        "src/util/attribute/attr.rs",
        "src/util/attribute/data.rs",
        "src/util/attribute/date.rs",
        "src/util/attribute/error.rs",
        "src/util/attribute/mod.rs",
        "src/util/ef.rs",
        "src/util/escape.rs",
        "src/util/helper.rs",
        "src/util/key/ecdsa.rs",
        "src/util/key/mod.rs",
        "src/util/key/rsa.rs",
        "src/util/mod.rs",
        "src/util/signing.rs",
    ],
    compile_data = [
        ":attribute_type",
        ":certificate_type",
        ":key_type",
        ":mechanism_type",
        ":object_class",
    ],
    crate_name = "hsmtool",
    rustc_env = {
        "ATTRIBUTE_TYPE": "$(location :attribute_type)",
        "CERTIFICATE_TYPE": "$(location :certificate_type)",
        "KEY_TYPE": "$(location :key_type)",
        "MECHANISM_TYPE": "$(location :mechanism_type)",
        "OBJECT_CLASS": "$(location :object_class)",
    },
    deps = [
        "//sw/host/hsmtool/acorn",
        "//sw/host/sphincsplus",
        "@crate_index//:anyhow",
        "@crate_index//:clap",
        "@crate_index//:cryptoki",
        "@crate_index//:cryptoki-sys",
        "@crate_index//:der",
        "@crate_index//:directories",
        "@crate_index//:ecdsa",
        "@crate_index//:hex",
        "@crate_index//:indexmap",
        "@crate_index//:log",
        "@crate_index//:once_cell",
        "@crate_index//:p256",
        "@crate_index//:pem-rfc7468",
        "@crate_index//:rand",
        "@crate_index//:regex",
        "@crate_index//:rsa",
        "@crate_index//:rustix",
        "@crate_index//:serde",
        "@crate_index//:serde_bytes",
        "@crate_index//:serde_json",
        "@crate_index//:sha2",
        "@crate_index//:strum",
        "@crate_index//:thiserror",
        "@crate_index//:typetag",
        "@crate_index//:zeroize",
        "@lowrisc_serde_annotate//serde_annotate",
    ],
)

rust_binary(
    name = "hsmtool",
    srcs = ["src/hsmtool.rs"],
    deps = [
        ":hsmlib",
        "@crate_index//:anyhow",
        "@crate_index//:clap",
        "@crate_index//:cryptoki",
        "@crate_index//:env_logger",
        "@crate_index//:log",
    ],
)

rust_test(
    name = "hsmlib_test",
    crate = ":hsmlib",
    data = glob([
        "testdata/**",
    ]),
    env = {
        "TESTDATA": "$(rootpath testdata/key/test1_pkcs1.der)",
    },
)

rust_doc(
    name = "hsmlib_doc",
    crate = ":hsmlib",
)
