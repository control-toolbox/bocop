#!/bin/bash

## shell script for Bocop build
## Pierre Martinon
## 2023

PROBLEM_PATH=${PWD}

## set build folder
mkdir -p build
cd build

## default cmake options
BOCOP_ROOT_PATH="../../../"
BUILDTYPE="Release"
COVERAGE="False"
EXEC="True"
WRAPPER="False"

## set specific cmake options
while getopts "cdew" options; do
case "${options}" in
c) COVERAGE="True";;
d) BUILDTYPE="Debug";;
e) EXEC="False";;
w) WRAPPER="True";;
esac
done

## cmake step
cmake -DCMAKE_BUILD_TYPE=${BUILDTYPE} -DCOVERAGE=${COVERAGE} -DEXEC=${EXEC} -DWRAPPER=${WRAPPER} -DPROBLEM_DIR=${PROBLEM_PATH} ${BOCOP_ROOT_PATH}
status=$?

## build step
if [ "$status" -eq "0" ]; then
cmake --build . -j
status=$?
fi

## back to problem folder
cd ${PROBLEM_PATH}

exit $status
