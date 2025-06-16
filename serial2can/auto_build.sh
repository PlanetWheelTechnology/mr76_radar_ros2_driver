#!/bin/bash
#Author: David Hu
#Date: 2022-09

# Exit on error
set -e

BUILD_MODE_DEBUG=1
BUILD_MODE_RELEASE=0

echo "Start cmake build"
read -p "Please select build mode [Debug(1)/Release(0)]:" BUILD_MODE

if [ ! -e "./build" ];then
  mkdir build
  echo "create ./build/"
fi

cd ./build

if [ ${BUILD_MODE} == ${BUILD_MODE_DEBUG} ];then
  cmake -DCMAKE_BUILD_TYPE=Debug  ..
elif [ ${BUILD_MODE} == ${BUILD_MODE_RELEASE} ];then
  cmake -DCMAKE_BUILD_TYPE=Release  ..
else
  echo "build mode input error"
  exit 0
fi

make
