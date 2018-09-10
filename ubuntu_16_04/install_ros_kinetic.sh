#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

echo -e $RESET $GREEN $NL"Setting up the sources list" $RESET
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Setting up the keys" $RESET
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Updating the package list" $RESET
sudo apt update

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Installing ROS" $RESET
sudo apt install ros-kinetic-desktop

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Initializing rosdep" $RESET
sudo rosdep init
rosdep update

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Adding ROS env setup to bashrc" $RESET
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Installing basic ROS tools" $RESET
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
source ~/.bashrc

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Initializing the catkin workspace" $RESET
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Finished installing ROS. Go catkin_make your workspace" $RESET
