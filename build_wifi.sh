#!/bin/bash
# Author: nAa @ xda
# Script for building the wifi modules after a kernel update
# the resulting modules are copied to pwd

WD=`pwd`
export KERNEL_DIR=$WD
pushd vendor_ti_wlan/sta/platforms/os/linux/
source msm_env_delta.bash
make clean 
make
cp *.ko $WD
popd
pushd vendor_ti_wlan/ap/platforms/os/linux/
make clean 
make
cp *.ko $WD
popd
