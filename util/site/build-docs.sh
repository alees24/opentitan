#!/usr/bin/env bash
# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

set -e

# Get the project directory from the location of this script
this_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
proj_root=$( realpath "${this_dir}/../.." )
build_dir="${proj_root}/build-site"

#######
# CLI #
#######
declare command="build"
# The following are only used for "command='serve-proxy'" ------------------------|
declare public_domain="" # (optional) The public-facing domain                  <-|
declare public_port=""   # (optional)The port added to the public-facing domain <-|
declare book_out="${build_dir}/book"

command="$1"
public_domain="${2}"
public_port="${3}"

case "$command" in
  "build"|"build-staging"|"build-local"|"serve"|"serve-proxy")
    ;;
  "help"|*)
    echo "USAGE: $0 <command> [public_domain] [public_port]"
    echo ""
    echo "commands:"
    echo "  help          prints this message."
    echo "  build         build the site and docs for prod"
    echo "  build-local   build the site and docs for a localhost server"
    echo "  build-staging build the site and docs for staging.opentitan.org"
    echo "  serve         build and serve the site locally"
    echo "  serve-proxy   build and serve the site, with a public url"
    echo ""
    echo "Optional arguments [public_domain] and [public_port] are only used when command='serve-proxy'"
    exit 0
    ;;
esac

#################
# CONFIGURATION #
#################
declare base_url
declare serve_port # The port used by the local webserver (using "build-docs.sh serve/serve-proxy")

getURLs () {
    # Defaults here are for production ("build-docs.sh build")
    local scheme="https"
    local domain="opentitan.org"
    local port=""

    # Use un-encrypted localhost URLs when building/serving locally
    # - serve on port 9000.
    if [ "$command" = "build-local" ] || \
       [ "$command" = "serve" ]; then
        scheme="http"
        domain="localhost"
        port=":9000"
        serve_port=":9000"
    fi
    # "serve-proxy" gives us some simple defaults for serving behind a proxy:
    # - set the public_domain/public_port appropriately (see $2/$3)
    # - serve on port 8000.
    if [ "$command" = "serve-proxy" ] ; then
        scheme="http"
        domain="${public_domain}"
        port=":${public_port}"
        serve_port=":8000"
    fi
    if [ "$command" = "build-staging" ] ; then
        scheme="https"
        domain="staging.opentitan.org"
    fi

    base_url="${scheme}://${domain}${port}"
}
getURLs

############
# BUILDING #
############

buildSite () {
    echo "Build Directory : ${build_dir}"
    mkdir -p "${build_dir}"
    mkdir -p "${build_dir}/gen/doxy"

    echo "Building doxygen..."
    # shellcheck disable=SC2086
    ./bazelisk.sh build --verbose_failures //doc:doxygen
    while read -r line; do
      cp -rf "$line" "${build_dir}/gen/"
    done < <(./bazelisk.sh cquery --output=files //doc:doxygen)
    # The files from bazel-out aren't writable. This ensures those that were copied are.
    chmod +w -R "${build_dir}/gen/"
    echo "Doxygen build complete."

    echo "Building mdBook..."
    ./bazelisk.sh build //doc:mdbook
    while read -r line; do
      cp -rf "$line" "${book_out}"
    done < <(./bazelisk.sh cquery --output=files //doc:mdbook)
    chmod +w -R "${book_out}"
    echo "mdBook build complete."

    # Build Rust Documentation
    local rustdoc_dir="${build_dir}/gen/rustdoc/"
    mkdir -p "${rustdoc_dir}"
    local bazel_out target_rustdoc target_rustdoc_output_path
    bazel_out="$(./bazelisk.sh info output_path 2>/dev/null)"
    for target_rustdoc in "sw/host/opentitanlib:opentitanlib_doc" "sw/host/hsmtool:hsmlib_doc" "sw/host/ot_certs:ot_certs_doc"
    do
      target_rustdoc_output_path="${bazel_out}/k8-fastbuild/bin/$(echo ${target_rustdoc} | tr ':' '/').rustdoc" #TODO : get the target's path using cquery
      ./bazelisk.sh build --experimental_convenience_symlinks=ignore "${target_rustdoc}"
      cp -rf "${target_rustdoc_output_path}"/* "${rustdoc_dir}"
      chown -R "$USER": "${rustdoc_dir}"
      chmod -R +w "${rustdoc_dir}"
    done
    # The files from bazel-out aren't writable. This ensures those that were copied are.
    chmod +w -R "${rustdoc_dir}"

    # Block diagram stats
    mkdir -p "${build_dir}/reports"
    python3 "${proj_root}/util/site/fetch_block_stats.py" "${build_dir}/reports/earlgrey-stats.json"

    rm -rf "${build_dir}/gen/api-xml" # Remove the intermediate XML that doxygen uses to generate HTML.
    # -------
}
buildSite

###########
# SERVING #
###########

# If serving, run the python HTTP server
if [ "$command" = "serve" ] || [ "$command" = "serve-proxy" ]; then
  echo "--------------------------------------------"
  echo
  echo "Website being served at : ${base_url}"
  echo
  echo "--------------------------------------------"
  python3 -m http.server -d "$build_dir" "${serve_port:1}"
                                         # strip leading :
fi
