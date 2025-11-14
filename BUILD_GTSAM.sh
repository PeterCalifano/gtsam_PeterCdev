#!bin/bash
# Script to build gtsam with MATLAB interface in GNU/Linux systems
# Note: this script assumes to be in the "gtsam" root folder.
# Created January 2024, modified May 2024 for Ubuntu 24.04 TLS by PeterC.
# Last updated with shell parser by PeterC, July 2024

# NOTE: If -r script is only allowed to change CXX flags and optionally include wrappers.
set -Euo pipefail # Exit on error in a pipeline

# Default values
build_path=build
install_path=install
is_default_build_path=true
jobs=3
rebuild=false
BUILD_TYPE=relwithdebinfo # default build type, possible options: debug, release, relwithdebinfo, minsizerel
ADD_CHECKS=false
ADD_CXX_FLAGS=""
WITH_PYTHON=false
WITH_MATLAB=false
WITH_UNSTABLE=false
WITH_EXPMAP=true
USE_TANGENT_PREINTEGR=true
PYTHON_EXE=$HOME/miniconda3/home/peterc/miniconda3/envs/gtsam/bin/python3.12 # Default assumes there is a conda environment in "$HOME/miniconda3"

# Parse options using getopt
# NOTE: no ":" after option means no argument, ":" means required argument, "::" means optional argument
OPTIONS=B::,j::,i::,r,t::,c,f::,p,m,u,e,o
LONGOPTIONS=build_path::,jobs::,install-path::,rebuild,type-build::,checks,flagsCXX::,python-wrap::,matlab-wrap::,unstable_build,exp_map_enabled,on_manifold_preintegr

# Parsed arguments list with getopt
PARSED=$(getopt --options ${OPTIONS} --longoptions ${LONGOPTIONS} --name "$0" -- "$@")

# Check validity of input arguments 
if [[ $? -ne 0 ]]; then
  # e.g. $? == 1
  #  then getopt has complained about wrong arguments to stdout
  exit 2
fi

# Parse arguments
eval set -- "$PARSED"

# Process options (change default values if needed)
while true; do
  case "$1" in
    -B|--build_path)
      if [ -n "$2" ] && [ "$2" != "--" ]; then # Check how many args (if 2)
        build_path="$2"
        is_default_build_path=false
        shift 2 # Shift of two args, i.e. $1 will then point to the next argument
      else 
      # Handle the default case (no optional argument provided), thus shift of 1
        build_path="build"
        is_default_build_path=true
        shift
      fi
      ;;
    -j|--jobs)
      if [ -n "$2" ] && [ "$2" != "--" ]; then
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
      if [ -n "$2" ] && [ "$2" != "--" ]; then
        BUILD_TYPE="$2"
        if [ "${BUILD_TYPE}" == "debug" ]; then
          ADD_CXX_FLAGS="${ADD_CXX_FLAGS} -Wall -Wextra"
        fi
        shift 2
      else
        BUILD_TYPE=debug
        ADD_CXX_FLAGS="${ADD_CXX_FLAGS} -Wall -Wextra"
        shift
      fi
      ;;
    -c|--checks)
      ADD_CHECKS=true
      shift
      ;;
    -f|--flagsCXX)
      if [ -n "$2" ] && [ "$2" != "--" ]; then
        ADD_CXX_FLAGS="$2"
        shift 2
      else 
        ADD_CXX_FLAGS="${ADD_CXX_FLAGS}"
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

if [ "${rebuild}" = true ]; then
  # REBUILDING FROM EXISTING BUILD
  if [ "${is_default_build_path}" = true ]; then
    build_path="build_dev" # Set default for rebuild if not specified
  fi   
  echo "REBUILDING GTSAM with options..."
  echo -e "\tbuild_path: $build_path"
  echo -e "\tJobs: $jobs"
  echo -e "\tBuild Type: ${BUILD_TYPE}"
  echo -e "\tEnforced compile flags: ${ADD_CXX_FLAGS}"
  echo -e "\tPython wrapper build: ${WITH_PYTHON}"
  echo -e "\tPython executable path: ${PYTHON_EXE}"
  echo -e "\tMATLAB wrapper build: ${WITH_MATLAB}"
  echo -e "\tBuild GTSAM unstable: ${WITH_UNSTABLE}"
  if ! [ -d $build_path ]; then
      echo "ERROR: NO PREVIOUS BUILD FOUND! EXITING..." >&2
      exit 1
  else

  sleep 1

  # Build and install (system-wide)
  if [ ${BUILD_TYPE} == "debug" ]; then
    cmake ${build_path} -DCMAKE_CXX_FLAGS=${ADD_CXX_FLAGS} \
    -DCMAKE_C_FLAGS=${ADD_CXX_FLAGS} \
    -DGTSAM_BUILD_PYTHON=${WITH_PYTHON} \
    -DPYTHON_EXECUTABLE=${PYTHON_EXE} \
    -DGTSAM_INSTALL_MATLAB_TOOLBOX=${WITH_MATLAB} \
    -DGTSAM_BUILD_UNSTABLE:OPTION=${WITH_UNSTABLE} \
    -DCMAKE_INSTALL_PREFIX=${install_path}

    make -j ${jobs} -C ${build_path} # |& tee MAKE_OUTPUT_LOG.txt
    if [ "${ADD_CHECKS}" = true ]; then
      make check -j ${jobs} -C ${build_path} # |& tee MAKE_CHECK_OUTPUT_LOG.txt
    fi
  else
    make check -j ${jobs} -C ${build_path} # |& tee MAKE_CHECK_OUTPUT_LOG.txt
    make install -j ${jobs} -C ${build_path} # |& tee MAKE_INSTALL_OUTPUT_LOG.txt
  fi

  fi

else
 # BUILDING FROM SCRATCH
  echo "Building with options..."
  echo -e "\tBuild path: ${build_path}"
  echo -e "\tInstall path: ${install_path}"
  echo -e "\tJobs: $jobs"
  echo -e "\tBuild Type: ${BUILD_TYPE}"
  echo -e "\tEnforced compile flags: ${ADD_CXX_FLAGS}"
  echo -e "\tPython wrapper build: ${WITH_PYTHON}"
  echo -e "\tMATLAB wrapper build: ${WITH_MATLAB}"
  echo -e "\tBuild GTSAM unstable: ${WITH_UNSTABLE}"
  echo -e "\tBuild GTSAM with expmap: ${WITH_EXPMAP}"
  echo -e "\tUsing Tangent Preintegration: ${USE_TANGENT_PREINTEGR}"
  sleep 1

  # Install dependencies (should add check if already installed)
  sudo apt update
  #udo apt install libboost1.74-all-dev cmake libtbb-dev -y
  #sudo apt install libboost-all-dev cmake libtbb-dev -y
  sudo apt install gcc-11 g++-11

  # NEED TO ADD IF "not installed" for the following:
  #sudo apt-get install python3-pip -y 
  #sudo apt-get install python-is-python3 -y
  #pip install pyparsing numpy 
  sudo apt-get install libeigen3-dev -y


  if  [ -d $build_path ]; then
      sudo rm -r ${build_path}/
  fi

  # Export path to use GCC 11.4 instead of >13.0
  #export CC=/usr/bin/gcc-11
  #export CXX=/usr/bin/g++-11

  # Generate makefiles with verbose output log
  cmake  -B ${build_path} -S . -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
          -DGTSAM_BUILD_UNSTABLE=${WITH_UNSTABLE} \
          -DGTSAM_BUILD_PYTHON=${WITH_PYTHON} -DGTSAM_INSTALL_MATLAB_TOOLBOX=${WITH_MATLAB} \
          -DGTSAM_WITH_TBB=ON \
          -DGTSAM_WITH_EIGEN_MKL=OFF \
  	      -DGTSAM_UNSTABLE_BUILD_PYTHON=${WITH_PYTHON} \
          -DCMAKE_CXX_FLAGS=${ADD_CXX_FLAGS} \
          -DCMAKE_C_FLAGS=${ADD_CXX_FLAGS} \
          -DGTSAM_TANGENT_PREINTEGRATION=${USE_TANGENT_PREINTEGR} \
          -DGTSAM_POSE3_EXPMAP=${WITH_EXPMAP} \
          -DGTSAM_ROT3_EXPMAP=${WITH_EXPMAP} \
          -DCMAKE_INSTALL_PREFIX=${install_path}
  #        -DCMAKE_VERBOSE_MAKEFILE=ON . |& tee CMAKE_OUTPUT_LOG.txt .. 

  # Build and install (system-wide)
  if [ ${BUILD_TYPE} == "debug" ]; then
    make -j ${jobs} -C ${build_path} # |& tee MAKE_OUTPUT_LOG.txt
    if [ "${ADD_CHECKS}" = true ]; then
      make check -j ${jobs} -C ${build_path} # |& tee MAKE_CHECK_OUTPUT_LOG.txt
    fi
  else
    make -j ${jobs} -C ${build_path} # |& tee MAKE_OUTPUT_LOG.txt
    make check -j ${jobs} -C ${build_path} # |& tee MAKE_CHECK_OUTPUT_LOG.txt
    make install -j ${jobs} -C ${build_path} # |& tee MAKE_INSTALL_OUTPUT_LOG.txt
  fi

fi
