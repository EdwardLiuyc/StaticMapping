#!/bin/bash

wget https://nchc.dl.sourceforge.net/project/geographiclib/distrib/GeographicLib-1.50.1.tar.gz
tar -zxvf GeographicLib-1.50.1.tar.gz && cd GeographicLib-1.50.1
rm GeographicLib-1.50.1.tar.gz
mkdir build && cd build
cmake ..
make -j4
sudo make install

exit 0