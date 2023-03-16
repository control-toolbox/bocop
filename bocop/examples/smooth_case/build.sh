#!/bin/bash

## shell script for Bocop build
## Pierre Martinon, Inria
## 2020

## set build folder
mkdir -p build
cd build

## default cmake options
BOCOP_ROOT_PATH="../../../"
buildtype="Release"

## set specific cmake options
while getopts ds option 
do
case "${option}" in
d) buildtype="Debug";;
esac
done

## launch cmake, make and go back to problem folder
KERNEL=`uname -s`
if [[ "$KERNEL" == *"MINGW"* ]]; then
cmake -G "MSYS Makefiles" -DCMAKE_BUILD_TYPE=${buildtype} -DPROBLEM_DIR=${PWD}/.. ${BOCOP_ROOT_PATH}
else
cmake -DCMAKE_BUILD_TYPE=${buildtype} -DPROBLEM_DIR=${PWD}/..  ${BOCOP_ROOT_PATH}
fi
status=$?
if [ "$status" -eq "0" ]; then
make -j
status=$?
fi
cd -

exit $status
