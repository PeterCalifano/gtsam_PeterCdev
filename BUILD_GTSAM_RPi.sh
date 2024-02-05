#!bin/bash
# Script to build gtsam with Python/MATLAB interface on RaspberryPi
# Note: this script assumes to be in the "gtsam" root folder.

# Install dependencies
sudo apt-get update
sudo apt-get install libboost-all-dev cmake libtbb-dev -y

# NEED TO ADD IF "not installed" for the following:
sudo apt-get install python3-pip -y 
sudo apt-get install python-is-python3 -y
pip install pyparsing numpy 
sudo apt-get install libeigen3-dev -y


buildpath="build"
if ! [ -d $buildpath ]; then
    mkdir build
    cd build
else
    echo "build directory exists, skipping creation, cleaning... "
    cd build 
    make clean
fi

# ADD check to see if in build else throw error

# Generate makefiles with verbose output log
cmake   .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DGTSAM_BUILD_UNSTABLE:OPTION=OFF -DGTSAM_BUILD_CONVENIENCE_LIBRARIES:OPTION=OFF \
        -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_WITH_EIGEN_MKL=OFF \
        -DCMAKE_VERBOSE_MAKEFILE=OFF |& tee output_file.txt  

# Build and install (system-wide)
sudo make check |& tee MAKE_CHECK_OUTPUT_LOG.txt
sudo make install |& tee MAKE_INSTALL_OUTPUT_LOG.txt
#make python-install -j4
