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

wget https://nchc.dl.sourceforge.net/project/geographiclib/distrib/GeographicLib-1.50.1.tar.gz
tar -zxvf GeographicLib-1.50.1.tar.gz && rm GeographicLib-1.50.1.tar.gz
cd GeographicLib-1.50.1
mkdir build && cd build
cmake .. && make -j$Jobs
sudo make install

exit 0