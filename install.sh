#!/bin/bash
PROJECT_DIR=$(pwd)
echo $PROJECT_DIR

cd $PROJECT_DIR/thirdparty/jsoncpp
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=ON
make -j
make install

cd $PROJECT_DIR
mkdir -p build
cd build
cmake ..
make -j
cd $PROJECT_DIR
