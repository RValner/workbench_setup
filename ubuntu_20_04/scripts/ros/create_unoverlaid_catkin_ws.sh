#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

OLD_CATKIN_WS_NAME=$ACTIVE_CATKIN_WS_NAME
NEW_CATKIN_WS_NAME=$1

echo -e $GREEN"Creating new unoverlaid catkin workspace to" $CATKIN_WS_PATH $RESET $STR_NEW_ESC

# Clear the package path variable and set the new ws as active
export ROS_PACKAGE_PATH=/opt/ros/melodic/setup.bash
export ACTIVE_CATKIN_WS_NAME=$NEW_CATKIN_WS_NAME
export ACTIVE_CATKIN_WS_PATH=$CATKIN_WS_PATH/$NEW_CATKIN_WS_NAME

# Create the catkin ws folder
mkdir -p $ACTIVE_CATKIN_WS_PATH/src
cd $ACTIVE_CATKIN_WS_PATH

# Modify the .bashrc
sed -i "s/$OLD_CATKIN_WS_NAME/$NEW_CATKIN_WS_NAME/g" ~/.bashrc

echo -e $GREEN"Done. Run catkin build" $RESET $STR_NEW_ESC
