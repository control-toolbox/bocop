#!/bin/bash

## shell script for Bocop build
## Pierre Martinon
## 2023

## set build folder
mkdir -p build
cd build

## default cmake options
BOCOP_ROOT_PATH="../../../"
BUIDTYPEE="Release"
COVERAGE="False"
NOWRAPPER="True"

## set specific cmake options
while getopts "cdw" options; do
case "${options}" in
c) COVERAGE="True";;
d) BUILDTYPE="Debug";;
w) NOWRAPPER="False";;
esac
done

## launch cmake, make and go back to problem folder
KERNEL=`uname -s`
if [[ "$KERNEL" == *"MINGW"* ]]; then
cmake -G "MSYS Makefiles" -DCMAKE_BUILD_TYPE=${BUILDTYPE} -DPROBLEM_DIR=${PWD}/.. ${BOCOP_ROOT_PATH}
else
cmake -DCMAKE_BUILD_TYPE=${BUILDTYPE} -DCOVERAGE=${COVERAGE} -DNOWRAPPER=${NOWRAPPER} -DPROBLEM_DIR=${PWD}/.. ${BOCOP_ROOT_PATH}
fi
status=$?
if [ "$status" -eq "0" ]; then
make -j
status=$?
fi
cd -

exit $status
