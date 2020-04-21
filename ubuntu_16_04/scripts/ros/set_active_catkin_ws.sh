g#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

OLD_CATKIN_WS_NAME=$ACTIVE_CATKIN_WS_NAME
NEW_CATKIN_WS_NAME=$1

echo -e $GREEN"Setting active catkin workspace to" $NEW_CATKIN_WS_NAME $RESET $STR_NEW_ESC

# Modify the .bashrc
sed -i "s/$OLD_CATKIN_WS_NAME/$NEW_CATKIN_WS_NAME/g" ~/.bashrc
source ~/.bashrc
cd $ACTIVE_CATKIN_WS_PATH

echo -e $GREEN"Done." $RESET $STR_NEW_ESC
