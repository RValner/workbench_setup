#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

#--------------------------------------------------------------#
#--------------------------------------------------------------#
#                                                              #
#                     FUNCTION DEFINITIONS                     #
#                                                              #
#--------------------------------------------------------------#
#--------------------------------------------------------------#

# Checks if previously executed process finished successfully or not
# Usage: check_success() <"Error message">
check_success() {
  if [[ $? != 0 ]]; then
    echo -e $RED$BOLD$1". Exiting"$RESET
    exit
  fi
}

# Usage: find_install_from_apt <package_name>
find_install_from_apt() {
  PACKAGE_NAME=$1
  
  # Look for the package
  SEARCH_RESULT=$(dpkg --list | grep $PACKAGE_NAME | awk -F: '{print $1}' | cut -d' ' -f3)

  if [[ $SEARCH_RESULT = $PACKAGE_NAME ]]; then
    echo -e $GREEN$BOLD"*" $PACKAGE_NAME $RESET$GREEN"package is already installed."$RESET
  else
    # Clone the rviz_plugin_manager package
    echo -e $RESET$GREEN"Installing" $PACKAGE_NAME $RESET
    sudo apt install $PACKAGE_NAME
  fi 
}

#--------------------------------------------------------------#
#--------------------------------------------------------------#
#                                                              #
#                     START OF THE SCRIPT                      #
#                                                              #
#--------------------------------------------------------------#
#--------------------------------------------------------------#

echo -e $RESET $GREEN $NL"Setting up the sources list" $RESET
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
check_success "Failed"

echo -e $RESET $GREEN $NL"Setting up the keys" $RESET
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
check_success "Failed"

echo -e $RESET $GREEN $NL"Updating the package list" $RESET
sudo apt update
check_success "Failed"

echo -e $RESET $GREEN $NL"Installing ROS" $RESET
find_install_from_apt ros-kinetic-desktop
check_success "Failed"

echo -e $RESET $GREEN $NL"Initializing rosdep" $RESET
sudo rosdep init
rosdep update
check_success "Failed"

echo -e $RESET $GREEN $NL"Adding ROS env setup to bashrc" $RESET
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
check_success "Failed"

echo -e $RESET $GREEN $NL"Installing basic ROS tools" $RESET
find_install_from_apt python-rosinstall
find_install_from_apt python-rosinstall-generator
find_install_from_apt python-wstool
find_install_from_apt build-essential
source ~/.bashrc
check_success "Failed"

echo -e $RESET $GREEN $NL"Initializing the catkin workspace" $RESET
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
check_success "Failed"

echo -e $RESET $GREEN $NL"Finished installing ROS. Go catkin_make your workspace" $RESET
