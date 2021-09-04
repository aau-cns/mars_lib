#!/bin/bash
echo "Copy Source Files..."
cp -r source/ test/
cd test && mkdir build && cd build
echo "Building the Appliaction..."
cmake ../ && make -j
echo "Running Google Tests"
make test
