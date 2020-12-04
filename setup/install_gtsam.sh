#!/bin/bash

set -o errexit
set -o verbose

git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam
git checkout tags/4.0.0-alpha2
mkdir build && cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j4
sudo make install

exit 0