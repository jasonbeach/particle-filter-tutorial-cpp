#!/bin/bash
set -e
script_dir="$(dirname "$(readlink -f "$0")")"
root_dir=$script_dir/..
DEFAULT_BUILD_TYPE=Release

main() {

  parse_args $@

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
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} "${SANITIZER_ARGS[@]}" \
    -DENABLE_WARNINGS_AS_ERRORS=OFF \
    -GNinja \
    -B $root_dir/build \
    -S $root_dir

  # build
  ninja -C $root_dir/build install
}

parse_args()
{
  BUILD_TYPE=$DEFAULT_BUILD_TYPE

  while getopts "sdh" opt; do
    case ${opt} in

      d )
        BUILD_TYPE=Debug
        ;;
      s )
        echo "enabling santizers"
        SANITIZER_ARGS=(-DCMAKE_CXX_FLAGS='-fno-omit-frame-pointer -fsanitize=address' -DCMAKE_EXE_LINKER_FLAGS='-fno-omit-frame-pointer -fsanitize=address')
        echo ${SANITIZER_ARGS}
        ;;
      h )
        usage
        exit
        ;;
      \? )
        usage
        exit
        ;;
    esac
  done
  shift $((OPTIND -1))


  echo "Build type: ${BUILD_TYPE}"

}

usage()
{
  cat <<EOF |

usage: ./build.sh [options]

options:

  -d: debug build
  -s: add santizers to build
EOF
  fmt -sw $(tput cols)
}

pushd $script_dir > /dev/null
main $@
popd > /dev/null
