#!/usr/bin/env zsh
# File        : local_setup.zsh
# Author      : Chandan Sheikder
# Email       : chandan@bit.edu.cn
# Phone       : +8618222390506
# Affiliation : Beijing Institute of Technology (BIT)
# Date        : 2026-03-23
#
# generated from catkin/cmake/templates/local_setup.zsh.in

CATKIN_SHELL=zsh

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
emulate -R zsh -c 'source "$_CATKIN_SETUP_DIR/setup.sh" --extend --local'
