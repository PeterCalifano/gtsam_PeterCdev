#!bin/bash
# Script to build gtsam with MATLAB interface in GNU/Linux systems
# Note: this script assumes to be in the "gtsam" root folder.
# Created January 2024, modified May 2024 for Ubuntu 24.04 TLS by PeterC.
# Last updated with shell parser by PeterC, July 2024

# NOTE: If -r script is only allowed to change CXX flags and optionally include wrappers.

# Default values
Buildpath="build" 
IS_BUILDPATH_DEFAULT=true
jobs=3
WITH_DYNAMICS_MODULE=false
install_deps=false
rebuild=false
BUILD_TYPE=debug # default build type, possible options: debug, release, relwithdebinfo, minsizerel
ADD_CHECKS=false
ADD_CXX_FLAGS=""
WITH_PYTHON=false
WITH_MATLAB=false
WITH_UNSTABLE=false
WITH_EXPMAP=true
USE_TANGENT_PREINTEGR=true

# Parse options using getopt
# NOTE: no ":" after option means no argument, ":" means required argument, "::" means optional argument
OPTIONS=B::,j::,w,i,r,t::,c,f::,p::,m::,u,e,o
LONGOPTIONS=Buildpath::,jobs::,with-dynamics-module,install-deps,rebuild,type-build::,checks,flagsCXX::,python-wrap::,matlab-wrap::,unstable_build,exp_map_enabled,on_manifold_preintegr

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
    -B|--Buildpath)
      if [ -n "$2" ] && [ "$2" != "--" ]; then # Check how many args (if 2)
        Buildpath="$2"
        IS_BUILDPATH_DEFAULT=false
        shift 2 # Shift of two args, i.e. $1 will then point to the next argument
      else 
      # Handle the default case (no optional argument provided), thus shift of 1
        Buildpath="build"
        IS_BUILDPATH_DEFAULT=true
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
    -w|--with-dynamics-module)
      WITH_DYNAMICS_MODULE=true
      shift
      ;;
    -i|--install-deps)
      install_deps=true
      shift
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
      if [ -n "$2" ] && [ "$2" != "--" ]; then
        WITH_PYTHON="$2"
        shift 2
      else
        WITH_PYTHON=false
        shift
      fi
      ;;
    -m|--matlab-wrap)  
      if [ -n "$2" ] && [ "$2" != "--" ]; then
        WITH_MATLAB="$2"
        shift 2
      else
        WITH_MATLAB=false
        shift
      fi
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
  if [ "${IS_BUILDPATH_DEFAULT}" = true ]; then
    Buildpath="build_dev" # Set default for rebuild if not specified
  fi   
  echo "REBUILDING GTSAM with options..."
  echo -e "\tBuildpath: $Buildpath"
  echo -e "\tJobs: $jobs"
  echo -e "\tBuild Type: ${BUILD_TYPE}"
  echo -e "\tEnforced compile flags: ${ADD_CXX_FLAGS}"
  echo -e "\tPython wrapper build: ${WITH_PYTHON}"
  echo -e "\tMATLAB wrapper build: ${WITH_MATLAB}"
  echo -e "\tWith Dynamics Module: ${WITH_DYNAMICS_MODULE}"
  echo -e "\tBuild GTSAM unstable: ${WITH_UNSTABLE}"
  if ! [ -d $Buildpath ]; then
      echo "ERROR: NO PREVIOUS BUILD FOUND! EXITING..." >&2
      exit 1
  else

  sleep 1

  #cmake  -B ${Buildpath} -S . -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
  #        -DGTSAM_BUILD_UNSTABLE:OPTION=OFF -DGTSAM_BUILD_CONVENIENCE_LIBRARIES:OPTION=ON \
  #        -DGTSAM_BUILD_PYTHON=${WITH_PYTHON} -DGTSAM_INSTALL_MATLAB_TOOLBOX=${WITH_MATLAB} \
  #        -DGTSAM_WITH_TBB=ON \
  #        -DGTSAM_WITH_EIGEN_MKL=OFF \
  #	       -DGTSAM_UNSTABLE_BUILD_PYTHON=${WITH_PYTHON} \
  #        -DGTSAM_WITH_DYNAMICS=${WITH_DYNAMICS_MODULE} \
  #        -DCMAKE_CXX_FLAGS=${ADD_CXX_FLAGS} \
  #        -DCMAKE_C_FLAGS=${ADD_CXX_FLAGS}

  # Build and install (system-wide)
  if [ ${BUILD_TYPE} == "debug" ]; then
    sudo cmake ${Buildpath} -DCMAKE_CXX_FLAGS=${ADD_CXX_FLAGS} \
    -DCMAKE_C_FLAGS=${ADD_CXX_FLAGS} \
    -DGTSAM_BUILD_PYTHON=${WITH_PYTHON} \
    -DGTSAM_INSTALL_MATLAB_TOOLBOX=${WITH_MATLAB} \
    -DGTSAM_BUILD_UNSTABLE:OPTION=${WITH_UNSTABLE} \

    sudo make -j ${jobs} -C ${Buildpath} # |& tee MAKE_OUTPUT_LOG.txt
    if [ "${ADD_CHECKS}" = true ]; then
      sudo make check -j ${jobs} -C ${Buildpath} # |& tee MAKE_CHECK_OUTPUT_LOG.txt
    fi
  else
    sudo make check -j ${jobs} -C ${Buildpath} # |& tee MAKE_CHECK_OUTPUT_LOG.txt
    sudo make install -j ${jobs} -C ${Buildpath} # |& tee MAKE_INSTALL_OUTPUT_LOG.txt
  fi

  fi

else
 # BUILDING FROM SCRATCH
  echo "Building with options..."
  echo -e "\tBuildpath: $Buildpath"
  echo -e "\tJobs: $jobs"
  echo -e "\tWith Dynamics Module: ${WITH_DYNAMICS_MODULE}"
  echo -e "\tInstall Dependencies: ${install_deps}"
  echo -e "\tBuild Type: ${BUILD_TYPE}"
  echo -e "\tEnforced compile flags: ${ADD_CXX_FLAGS}"
  echo -e "\tPython wrapper build: ${WITH_PYTHON}"
  echo -e "\tMATLAB wrapper build: ${WITH_MATLAB}"
  echo -e "\tBuild GTSAM unstable: ${WITH_UNSTABLE}"
  echo -e "\tBuild GTSAM with expmap: ${WITH_EXPMAP}"
  echo -e "\tUsing Tangent Preintegration: ${USE_TANGENT_PREINTEGR}"
  sleep 1

  # Install dependencies (should add check if already installed)
  if [ "${install-deps}" = true ]; then
    sudo apt update
    sudo apt install libboost1.74-all-dev cmake libtbb-dev -y
    #sudo apt install libboost-all-dev cmake libtbb-dev -y
    sudo apt install gcc-11 g++-11

    # NEED TO ADD IF "not installed" for the following:
    #sudo apt-get install python3-pip -y 
    #sudo apt-get install python-is-python3 -y
    #pip install pyparsing numpy 
    sudo apt-get install libeigen3-dev -y
  fi


  if  [ -d $Buildpath ]; then
      sudo rm -r ${Buildpath}/
  fi

  # Export path to use GCC 11.4 instead of >13.0
  export CC=/usr/bin/gcc-11
  export CXX=/usr/bin/g++-11

  # Generate makefiles with verbose output log
  cmake  -B ${Buildpath} -S . -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
          -DGTSAM_BUILD_UNSTABLE=${WITH_UNSTABLE} -DGTSAM_BUILD_CONVENIENCE_LIBRARIES=OFF \
          -DGTSAM_BUILD_PYTHON=${WITH_PYTHON} -DGTSAM_INSTALL_MATLAB_TOOLBOX=${WITH_MATLAB} \
          -DGTSAM_WITH_TBB=ON \
          -DGTSAM_WITH_EIGEN_MKL=OFF \
  	      -DGTSAM_UNSTABLE_BUILD_PYTHON=${WITH_PYTHON} \
          -DGTSAM_WITH_DYNAMICS=${WITH_DYNAMICS_MODULE} \
          -DCMAKE_CXX_FLAGS=${ADD_CXX_FLAGS} \
          -DCMAKE_C_FLAGS=${ADD_CXX_FLAGS} \
          -DGTSAM_TANGENT_PREINTEGRATION=${USE_TANGENT_PREINTEGR} \
          -DGTSAM_POSE3_EXPMAP=${WITH_EXPMAP} \
          -DGTSAM_ROT3_EXPMAP=${WITH_EXPMAP} 
  #        -DCMAKE_VERBOSE_MAKEFILE=ON . |& tee CMAKE_OUTPUT_LOG.txt .. 

  # Build and install (system-wide)
  if [ ${BUILD_TYPE} == "debug" ]; then
    sudo make -j ${jobs} -C ${Buildpath} # |& tee MAKE_OUTPUT_LOG.txt
    if [ "${ADD_CHECKS}" = true ]; then
      sudo make check -j ${jobs} -C ${Buildpath} # |& tee MAKE_CHECK_OUTPUT_LOG.txt
    fi
  else
    sudo make -j ${jobs} -C ${Buildpath} # |& tee MAKE_OUTPUT_LOG.txt
    sudo make check -j ${jobs} -C ${Buildpath} # |& tee MAKE_CHECK_OUTPUT_LOG.txt
    #sudo make install -j ${jobs} -C ${Buildpath} # |& tee MAKE_INSTALL_OUTPUT_LOG.txt
  fi

fi