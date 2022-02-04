#!/bin/bash

set -e

cmake --build build -- -j $(nproc)
exit 0
