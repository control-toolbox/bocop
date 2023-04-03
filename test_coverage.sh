#!/bin/bash

## shell script for Bocop code coverage test
## Pierre Martinon
## 2023

## use fuller problem built in debug/coverage mode
cd ./bocop/examples/fuller
rm -r build
./build.sh -cd
./bocopApp
mkdir -p coverage
lcov --version
lcov -c -d ./build/src/CMakeFiles/bocopApp.dir -o coverage/bocopApp.info
cd coverage
genhtml bocopApp.info
firefox index.html
cd ../../../..
