#!/bin/bash

Jobs=$(nproc)

while getopts j: flag
do
    case "${flag}" in
        j) Jobs=${OPTARG};;
    esac
done
echo "Jobs: $Jobs";

set -o errexit
set -o verbose

git clone https://github.com.cnpmjs.org/borglab/gtsam.git
cd gtsam
git checkout tags/4.0.3
mkdir build && cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_DOC_HTML=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_INSTALL_GEOGRAPHICLIB=OFF ..
make -j$Jobs
sudo make install
cd ../.. && rm -rf gtsam

exit 0