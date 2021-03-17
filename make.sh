#!/bin/bash

cmake --build build -- -j $(nproc)
exit 0
