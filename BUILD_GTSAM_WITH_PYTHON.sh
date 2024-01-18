#!bin/bash
# Script to build gtsam with Python/MATLAB interface in GNU/Linux systems
# Note: this script assumes to be in the "gtsam" root folder.

# Install dependencies
sudo apt-get update
sudo apt-get install libboost-all-dev cmake -y
sudo apt-get install python3-pip -y # NEED TO ADD IF "not installed"
sudo apt-get install python-is-python3 # NEED TO ADD IF "not installed"
pip install pyparsing numpy 

buildpath=build
if [ -d "$buildpath" ]; then
mkdir build
else
echo "build directory exists, skipping creation..."
fi

# Change to build folder
cd build

# Generate makefiles
cmake   -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_UNSTABLE:OPTION=ON \
        -DGTSAM_BUILD_PYTHON=ON \ 
        -DGTSAM_INSTALL_MATLAB_TOOLBOX=ON .. 

# Build and install (system-wide)
make check
sudo make install
sudo make python-install
