#! /bin/bash

export LD_LIBRARY_PATH=/home/peter/code/AMP-Tools-public/install/ubuntu20/lib:$LD_LIBRARY_PATH
echo $LD_LIBRARY_PATH
mkdir -p build && cd build
cmake ./.. -DAMP_BUILD_LIB=OFF
make
if [ $? != 0 ]; then
    echo "Build Failed!"
fi