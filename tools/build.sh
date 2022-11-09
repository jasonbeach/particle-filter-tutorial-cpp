#!/bin/bash

main() {
  root_dir=$script_dir/..

  # make build dir
  mkdir -p $root_dir/build

  # install conan packages
  conan install \
    -if $root_dir/build \
    -s compiler.libcxx=libstdc++11 \
    --build=missing \
    $root_dir/conanfile.py

  # configure
  cmake \
    -DCMAKE_TOOLCHAIN_FILE=$root_dir/build/conan_paths.cmake \
    -DCMAKE_INSTALL_PREFIX=$root_dir/install \
    -GNinja \
    -B $root_dir/build \
    -S $root_dir

  # build
  ninja -C $root_dir/build install
}

set -e
script_dir="$(dirname "$(readlink -f "$0")")"
pushd $script_dir > /dev/null
main $@
popd > /dev/null
