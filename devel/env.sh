#!/usr/bin/env sh
# File        : env.sh
# Author      : Chandan Sheikder
# Email       : chandan@bit.edu.cn
# Phone       : +8618222390506
# Affiliation : Beijing Institute of Technology (BIT)
# Date        : 2026-03-23
#
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# ensure to not use different shell type which was set before
CATKIN_SHELL=sh

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(cd "`dirname "$0"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"
exec "$@"
