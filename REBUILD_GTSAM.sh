#!bin/bash
# Script to re-build gtsam with Python/MATLAB interface in GNU/Linux systems
# Note: this script assumes to be in the "gtsam" root folder and that GTSAM has been previously installed from source

buildpath="build"

if ! [ -d $buildpath ]; then
    echo "ERROR: NO PREVIOUS BUILD FOUND! EXITING..." >&2
    exit 1
else
    cd build 

# ADD check to see if in build else throw error

# Generate makefiles with verbose output log
cmake   -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DGTSAM_BUILD_UNSTABLE:OPTION=OFF -DGTSAM_BUILD_CONVENIENCE_LIBRARIES:OPTION=ON \
        -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=ON \
        -DGTSAM_WITH_TBB=ON \
        -DGTSAM_WITH_EIGEN_MKL=OFF \
        -DCMAKE_VERBOSE_MAKEFILE=ON . 2>&1 | tee CMAKE_OUTPUT_LOG.txt .. 

# Build and install (system-wide)
sudo make check -j4
sudo make install -j4
#make python-install -j4

fi




