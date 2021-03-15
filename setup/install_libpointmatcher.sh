#!/bin/bash

set -o errexit
set -o verbose

git clone https://github.com.cnpmjs.org/ethz-asl/libpointmatcher.git
cd libpointmatcher
git checkout tags/1.3.1
mkdir build && cd build
cmake ..
make -j4
sudo make install

exit 0