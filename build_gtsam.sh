#!/usr/bin/env bash
# Script to build gtsam with MATLAB/Python interfaces on GNU/Linux.
# Note: this script assumes it is run from the gtsam root folder.
# Created January 2024, modified May 2024 for Ubuntu 24.04 TLS by PeterC.
# Last updated with shell parser by PeterC, July 2024. Cleaned up for clarity.

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: BUILD_GTSAM.sh [options]
  -B, --build_path [path]        Build directory (default: build; build_dev when rebuilding)
  -i, --install-path PATH        Install prefix (default: install)
  -j, --jobs [N]                 Parallel build jobs (default: 3, or 4 if flag provided without value)
  -r, --rebuild                  Reuse an existing build directory instead of starting clean
  -t, --type-build [TYPE]        debug | release | relwithdebinfo | minsizerel | test (default: relwithdebinfo)
  -c, --checks                   Run `make check` (also auto-enabled for release/test types)
  -f, --flagsCXX [FLAGS]         Extra C/C++ compiler flags
  -p, --python-wrap              Build Python wrapper
  -m, --matlab-wrap              Build MATLAB wrapper
  -u, --unstable_build           Build unstable components
  -e, --exp_map_disabled         Disable Expmap (Pose3/Rot3)
  -o, --on_manifold_preintegr    Use on-manifold preintegration instead of tangent
  -h, --help                     Show this help and exit

Environment:
  PYTHON_EXE     Override Python executable for wrappers (default: $HOME/miniconda3/envs/gtsam/bin/python3.12)
EOF
}

# Defaults
build_path="build"
install_path="install"
is_default_build_path=true
jobs=3
rebuild=false
BUILD_TYPE="relwithdebinfo"
ADD_CHECKS=false
RUN_CHECKS=false
ADD_CXX_FLAGS=""
WITH_PYTHON=false
WITH_MATLAB=false
WITH_UNSTABLE=false
WITH_EXPMAP=true
USE_TANGENT_PREINTEGR=true
PYTHON_EXE="${PYTHON_EXE:-$HOME/miniconda3/envs/gtsam/bin/python3.12}"

# Option parsing
OPTIONS=B::,j::,i::,r,t::,c,f::,p,m,u,e,o,h
LONGOPTIONS=build_path::,jobs::,install-path::,rebuild,type-build::,checks,flagsCXX::,python-wrap,matlab-wrap,unstable_build,exp_map_disabled,on_manifold_preintegr,help
PARSED=$(getopt --options "${OPTIONS}" --longoptions "${LONGOPTIONS}" --name "$0" -- "$@") || exit 2
eval set -- "${PARSED}"

while true; do
  case "$1" in
    -B|--build_path)
      if [ -n "${2-}" ] && [ "$2" != "--" ]; then
        build_path="$2"
        is_default_build_path=false
        shift 2
      else
        shift
      fi
      ;;
    -j|--jobs)
      if [ -n "${2-}" ] && [ "$2" != "--" ]; then
        jobs="$2"
        shift 2
      else
        jobs=4
        shift
      fi
      ;;
    -i|--install-path)
      install_path="$2"
      shift 2
      ;;
    -r|--rebuild)
      rebuild=true
      shift
      ;;
    -t|--type-build)
      if [ -n "${2-}" ] && [ "$2" != "--" ]; then
        BUILD_TYPE="$2"
        shift 2
      else
        BUILD_TYPE="debug"
        shift
      fi
      ;;
    -c|--checks)
      ADD_CHECKS=true
      shift
      ;;
    -f|--flagsCXX)
      if [ -n "${2-}" ] && [ "$2" != "--" ]; then
        ADD_CXX_FLAGS="$2"
        shift 2
      else
        shift
      fi
      ;;
    -p|--python-wrap)
      WITH_PYTHON=true
      shift
      ;;
    -m|--matlab-wrap)
      WITH_MATLAB=true
      shift
      ;;
    -u|--unstable_build)
      WITH_UNSTABLE=true
      shift
      ;;
    -e|--exp_map_disabled)
      WITH_EXPMAP=false
      shift
      ;;
    -o|--on_manifold_preintegr)
      USE_TANGENT_PREINTEGR=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "Not a valid option: $1" >&2
      exit 3
      ;;
  esac
done

BUILD_TYPE=$(echo "${BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')
case "${BUILD_TYPE}" in
  debug)
    ADD_CXX_FLAGS="${ADD_CXX_FLAGS:+${ADD_CXX_FLAGS} }-Wall -Wextra"
    ;;
  release|relwithdebinfo|minsizerel|test)
    ;;
  *)
    echo "Unsupported build type: ${BUILD_TYPE}" >&2
    exit 4
    ;;
esac

if [ "${ADD_CHECKS}" = true ]; then
  RUN_CHECKS=true
fi
case "${BUILD_TYPE}" in
  release|relwithdebinfo|minsizerel|test)
    RUN_CHECKS=true
    ;;
esac

if [ "${rebuild}" = true ] && [ "${is_default_build_path}" = true ]; then
  build_path="build_dev"
fi

log_config() {
  echo "Configured build:"
  echo "  build_path: ${build_path}"
  echo "  install_path: ${install_path}"
  echo "  jobs: ${jobs}"
  echo "  build type: ${BUILD_TYPE}"
  echo "  enforced compile flags: ${ADD_CXX_FLAGS}"
  echo "  run checks: ${RUN_CHECKS}"
  echo "  python wrapper: ${WITH_PYTHON}"
  if [ "${WITH_PYTHON}" = true ]; then
    echo "  python executable: ${PYTHON_EXE}"
  fi
  echo "  MATLAB wrapper: ${WITH_MATLAB}"
  echo "  unstable modules: ${WITH_UNSTABLE}"
  echo "  use Expmap: ${WITH_EXPMAP}"
  echo "  tangent preintegration: ${USE_TANGENT_PREINTEGR}"
}

configure_cmake() {
  local -a cmake_args=(
    -S .
    -B "${build_path}"
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
    -DGTSAM_BUILD_UNSTABLE="${WITH_UNSTABLE}"
    -DGTSAM_BUILD_PYTHON="${WITH_PYTHON}"
    -DGTSAM_INSTALL_MATLAB_TOOLBOX="${WITH_MATLAB}"
    -DGTSAM_WITH_TBB=ON
    -DGTSAM_WITH_EIGEN_MKL=OFF
    -DGTSAM_UNSTABLE_BUILD_PYTHON="${WITH_PYTHON}"
    -DCMAKE_CXX_FLAGS="${ADD_CXX_FLAGS}"
    -DCMAKE_C_FLAGS="${ADD_CXX_FLAGS}"
    -DGTSAM_TANGENT_PREINTEGRATION="${USE_TANGENT_PREINTEGR}"
    -DGTSAM_POSE3_EXPMAP="${WITH_EXPMAP}"
    -DGTSAM_ROT3_EXPMAP="${WITH_EXPMAP}"
    -DCMAKE_INSTALL_PREFIX="${install_path}"
  )

  if [ "${WITH_PYTHON}" = true ]; then
    cmake_args+=("-DPYTHON_EXECUTABLE=${PYTHON_EXE}")
    if [ ! -x "${PYTHON_EXE}" ]; then
      echo "Warning: PYTHON_EXE does not point to an executable: ${PYTHON_EXE}" >&2
    fi
  fi

  cmake "${cmake_args[@]}"
}

build_targets() {
  local target="${1:-all}"
  cmake --build "${build_path}" --target "${target}" --parallel "${jobs}"
}

run_checks_if_requested() {
  if [ "${RUN_CHECKS}" = true ]; then
    build_targets check
  fi
}

install_if_applicable() {
  if [ "${BUILD_TYPE}" != "debug" ]; then
    build_targets install
  fi
}

log_config

if [ "${rebuild}" = true ]; then
  if [ ! -d "${build_path}" ]; then
    echo "ERROR: No existing build directory at ${build_path}. Exiting..." >&2
    exit 1
  fi
  configure_cmake
  build_targets
  run_checks_if_requested
  install_if_applicable
else
  sudo apt update
  sudo apt install -y gcc-11 g++-11 libeigen3-dev

  if [ -d "${build_path}" ]; then
    rm -rf -- "${build_path}"
  fi

  configure_cmake
  build_targets
  run_checks_if_requested
  install_if_applicable
fi
