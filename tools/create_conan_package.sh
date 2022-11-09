#!/bin/bash

set -e
script_dir="$(dirname "$(readlink -f "$0")")"
root_dir=$script_dir/..

#options
PACKAGE_NAME=raptor_outerface
VERSION_FILE_NAME=version
DO_UPLOAD=false
FORCE_PACKAGE_BUILD=false


#compute other values
VERSION=$(cat $root_dir/${VERSION_FILE_NAME})
echo "${PACKAGE_NAME} VERSION: ${VERSION}"

COMMITS_SINCE_RELEASE=$(git rev-list v${VERSION}..HEAD --count)
echo "commits since v${VERSION} tag: ${COMMITS_SINCE_RELEASE}"

FULL_PACKAGE_NAME=${PACKAGE_NAME}/${VERSION}
echo "full package name: ${FULL_PACKAGE_NAME}"

main() {

  parse_args $@

  if [ ${COMMITS_SINCE_RELEASE} -gt "0" ]; then
    char=$((${COMMITS_SINCE_RELEASE} + 96))
    SUFFIX=$(echo ${char} | awk '{printf "%c", $1}')
    FULL_PACKAGE_NAME=${FULL_PACKAGE_NAME}${SUFFIX}
  fi

  check_git_clean

  conan create \
    -s compiler.libcxx=libstdc++11 \
    --build=missing \
    $root_dir \
    ${FULL_PACKAGE_NAME}@ssci/test

  conan create \
    -s compiler.libcxx=libstdc++11 -s build_type=Debug \
    --build=missing \
    $root_dir \
    ${FULL_PACKAGE_NAME}@ssci/test
  
  if [ "${DO_UPLOAD}" == true ]; then
   upload_package
  fi

}

check_git_clean(){

  if [[ `git status --porcelain` ]]; then
    
    if [ "${FORCE_PACKAGE_BUILD}" == false ]; then
      echo "WARNING: Git repository has uncommitted changes. Package will not be uploadable. Commit or stash changes or re-run with option -f to force build"
      exit 1;
    else
      echo "WARNING: Git repository has uncommitted changes. Package will not be uploadable."
    fi
  fi
}

upload_package(){
  

  if [ ${COMMITS_SINCE_RELEASE} == 0 ]; then
    if [ $(git status --porcelain | wc -l) -eq "0" ]; then
      echo "uploading package ${FULL_PACKAGE_NAME}"
      conan copy ${FULL_PACKAGE_NAME}@ssci/test ssci/stable --all --force
      conan upload ${FULL_PACKAGE_NAME}@ssci/stable -r ssci --all --confirm
    else
      echo "Cannot upload. Repository has changes that aren't committed."
    fi
  else
    echo "Cannot upload.  Not a tagged release."
    exit 1
  fi
}

parse_args() {
# most of these args aren't wired up.

  while getopts "fuh" opt; do
    case ${opt} in
      f)
        FORCE_PACKAGE_BUILD=true
        ;;
      u)
        DO_UPLOAD=true
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
  
}

usage() {
	cat <<EOF |

usage: ./create_conan_package.sh [options]

options:
  -f: force build package when git repo is dirty
  -u: upload package to ssci artifactory
  -h: display this help

EOF
	fmt -sw $(tput cols)
}


pushd $script_dir > /dev/null
main $@
popd > /dev/null
