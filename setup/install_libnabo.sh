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

git clone https://github.com/ethz-asl/libnabo.git
cd libnabo
git checkout tags/1.0.7
mkdir build && cd build
cmake .. && make -j$Jobs
sudo make install

exit 0
