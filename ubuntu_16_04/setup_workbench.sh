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
  SEARCH_RESULT=$(dpkg-query -W -f='${Status}' $PACKAGE_NAME 2>/dev/null | grep -c "ok installed")

  if [[ $SEARCH_RESULT -eq 1 ]]; then
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

ROOT_DIR=$(pwd)

# Update the package list
# ----------------------------------------------------

echo -e $RESET $GREEN $NL"Updating the package list" $RESET
sudo apt update

# Install and configure i3
# ----------------------------------------------------

find_install_from_apt i3
check_success "Failed to install i3"

echo -e $RESET $GREEN $NL"Configuring i3" $RESET
mkdir ~/.i3
cp $ROOT_DIR/configs/i3/general/config ~/.i3/config
check_success "Failed to configure i3"

mkdir -p ~/.config/i3status
cp $ROOT_DIR/configs/i3/status/config ~/.config/i3status/config
check_success "Failed to configure i3"

# Install and configure ranger
# ----------------------------------------------------

find_install_from_apt ranger
check_success "Failed to install ranger"

echo -e $RESET $GREEN $NL"Configuring ranger" $RESET
ranger --copy-config=all
cp $ROOT_DIR/configs/ranger/rc.conf ~/.config/ranger/rc.conf

# Install and configure conky
# ----------------------------------------------------

find_install_from_apt conky-all
check_success "Failed to install conky"

echo -e $RESET $GREEN $NL"Configuring conky" $RESET
mkdir ~/.config/conky
cp $ROOT_DIR/configs/conky/conky.conf ~/.config/conky/conky.conf
check_success "Failed to configure conky"

# Install other packages
# ----------------------------------------------------

find_install_from_apt shutter
find_install_from_apt feh
find_install_from_apt gedit-plugins
find_install_from_apt redshift

# Copy the scripts and resources
# ----------------------------------------------------

echo -e $RESET $GREEN $NL"Copying the scripts" $RESET
cp -r $ROOT_DIR/scripts/ ~/.custom_scripts/
cp -r $ROOT_DIR/resources/ ~/Pictures/
