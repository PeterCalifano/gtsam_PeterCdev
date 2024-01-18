#!bin/bash
# Script to build gtsam with Python/MATLAB interface in GNU/Linux systems
# Note: this script assumes to be in the "gtsam" root folder.

# Install dependencies
sudo apt-get update
sudo apt-get install libboost-all-dev cmake libtbb-dev -y
# NEED TO ADD IF "not installed" for the following:
sudo apt-get install python3-pip -y 
sudo apt-get install python-is-python3 
pip install pyparsing numpy 
sudo apt-get install libeigen3-dev 


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

# Generate makefiles
cmake   -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DGTSAM_BUILD_UNSTABLE:OPTION=ON -DGTSAM_BUILD_CONVENIENCE_LIBRARIES:OPTION=ON \
        -DGTSAM_BUILD_PYTHON=ON -DGTSAM_INSTALL_MATLAB_TOOLBOX=ON \
        -DGTSAM_WITH_TBB =ON \
        -DGTSAM_WITH_EIGEN_MKL=OFF .. 

# Build and install (system-wide)
make check
sudo make install
sudo make python-install
