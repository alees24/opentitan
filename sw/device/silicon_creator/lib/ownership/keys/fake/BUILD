# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

load("//rules/opentitan:keyutils.bzl", "key_ecdsa", "key_sphincs_plus")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "includes",
    testonly = True,
    hdrs = [
        "activate_ecdsa_p256.h",
        "app_dev_ecdsa_p256.h",
        "app_dev_spx.h",
        "app_prod_ecdsa_p256.h",
        "app_prod_spx.h",
        "app_test_ecdsa_p256.h",
        "owner_ecdsa_p256.h",
        "unlock_ecdsa_p256.h",
    ],
)

cc_library(
    name = "fake",
    srcs = [
        "ownership_ecdsa_keys_fake.c",
    ],
    hdrs = [
        "no_owner_recovery_ecdsa_p256.h",
    ],
    deps = [
        "//sw/device/silicon_creator/lib/ownership:ownership_key",
    ],
    alwayslink = True,
)

filegroup(
    name = "no_owner_recovery_key",
    srcs = ["no_owner_recovery_ecdsa_p256.der"],
)

filegroup(
    name = "owner_key",
    srcs = ["owner_ecdsa_p256.der"],
)

filegroup(
    name = "owner_key_pub",
    srcs = ["owner_ecdsa_p256.pub.der"],
)

filegroup(
    name = "activate_key",
    srcs = ["activate_ecdsa_p256.der"],
)

filegroup(
    name = "unlock_key",
    srcs = ["unlock_ecdsa_p256.der"],
)

filegroup(
    name = "app_ecdsa_prod",
    srcs = ["app_prod_ecdsa_p256.der"],
)

filegroup(
    name = "app_prod_ecdsa_pub",
    srcs = ["app_prod_ecdsa_p256.pub.der"],
)

filegroup(
    name = "app_dev",
    srcs = ["app_dev_ecdsa_p256.der"],
)

filegroup(
    name = "app_dev_pub",
    srcs = ["app_dev_ecdsa_p256.pub.der"],
)

key_ecdsa(
    name = "app_prod_ecdsa",
    config = "EcdsaP256",
    method = "local",
    private_key = "app_prod_ecdsa_p256.der",
    pub_key = "app_prod_ecdsa_p256.pub.der",
    type = "ProdKey",
)

key_sphincs_plus(
    name = "app_prod_spx",
    config = "Sha2128s",
    method = "local",
    private_key = "app_prod_spx.pem",
    pub_key = "app_prod_spx.pub.pem",
    type = "ProdKey",
)

key_ecdsa(
    name = "app_dev_ecdsa",
    config = "EcdsaP256",
    method = "local",
    private_key = "app_dev_ecdsa_p256.der",
    pub_key = "app_dev_ecdsa_p256.pub.der",
    type = "DevKey",
)

key_sphincs_plus(
    name = "app_dev_spx",
    # TODO(cfrantz): Change this to Prehash after putting
    # the prehash infrastructure in place.
    config = "Sha2128s",
    method = "local",
    private_key = "app_dev_spx.pem",
    pub_key = "app_dev_spx.pub.pem",
    type = "DevKey",
)

key_ecdsa(
    name = "app_test_ecdsa",
    config = "EcdsaP256",
    method = "local",
    private_key = "app_test_ecdsa_p256.der",
    pub_key = "app_test_ecdsa_p256.pub.der",
    type = "TestKey",
)

key_ecdsa(
    name = "app_unauthorized_ecdsa",
    config = "EcdsaP256",
    method = "local",
    private_key = "app_unauthorized_ecdsa_p256.der",
    pub_key = "app_unauthorized_ecdsa_p256.pub.der",
    type = "TestKey",
)
