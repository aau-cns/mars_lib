#!/bin/bash
echo "Copy Source Files..."
cp -r source/ test/
cd test
echo "Delete temporary test files to enforce unzip of original data..."
rm -rf source/tests/test_data
echo "Building the Appliaction..."
mkdir build && cd build
cmake ../ && make -j
echo "Running Google Tests"
make test
