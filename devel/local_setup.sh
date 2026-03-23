#!/usr/bin/env sh
# File        : local_setup.sh
# Author      : Chandan Sheikder
# Email       : chandan@bit.edu.cn
# Phone       : +8618222390506
# Affiliation : Beijing Institute of Technology (BIT)
# Date        : 2026-03-23
#
# generated from catkin/cmake/template/local_setup.sh.in

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time
: ${_CATKIN_SETUP_DIR:=/catkin_ws/devel}
CATKIN_SETUP_UTIL_ARGS="--extend --local"
. "$_CATKIN_SETUP_DIR/setup.sh"
unset CATKIN_SETUP_UTIL_ARGS
