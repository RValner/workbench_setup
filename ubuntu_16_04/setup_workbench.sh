#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

ROOT_DIR=$(pwd)

# # # # # # # # # # # # # #
#
# Update the package list
#
# # # # # # # # # # # # # #

echo -e $RESET $GREEN $NL"Updating the package list" $RESET
sudo apt update

# # # # # # # # # # # # # #
#
# Install and configure i3
#
# # # # # # # # # # # # # #

echo -e $RESET $GREEN $NL"Installing i3" $RESET
sudo apt install i3

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed to install i3. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Configuring i3" $RESET
mkdir ~/.i3
cp $ROOT_DIR/configs/i3/general/config ~/.i3/config

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed to configure i3. Exiting"$RESET
  exit
fi

mkdir -p ~/.config/i3status
cp $ROOT_DIR/configs/i3/status/config ~/.config/i3status/config

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed to configure i3. Exiting"$RESET
  exit
fi

# # # # # # # # # # # # # # # #
#
# Install and configure ranger
#
# # # # # # # # # # # # # # # #

sudo apt install ranger

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed to install ranger. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Configuring ranger" $RESET
ranger --copy-config=all
cp $ROOT_DIR/configs/ranger/rc.conf ~/.config/ranger/rc.conf

# # # # # # # # # # # # # # # #
#
# Install and configure conky
#
# # # # # # # # # # # # # # # #

echo -e $RESET $GREEN $NL"Installing conky" $RESET
sudo apt install conky-all

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed to install conky. Exiting"$RESET
  exit
fi

echo -e $RESET $GREEN $NL"Configuring conky" $RESET
mkdir ~/.config/conky
cp $ROOT_DIR/configs/conky/conky.conf ~/.config/conky/conky.conf

# Check if the installation was successful or not
if [[ $? != 0 ]]; then
  echo -e $RED $BOLD"Failed to configure conky. Exiting"$RESET
  exit
fi

# # # # # # # # # # # # # # # #
#
# Install other packages
#
# # # # # # # # # # # # # # # #

echo -e $RESET $GREEN $NL"Installing shutter" $RESET
sudo apt install shutter feh gedit-plugins

# # # # # # # # # # # # # # # # #
#
# Copy the scripts and resources
#
# # # # # # # # # # # # # # # # #

echo -e $RESET $GREEN $NL"Copying the scripts" $RESET
mkdir -p ~/.custom_scripts
cp $ROOT_DIR/scripts/toggle-xkbmap.sh ~/.custom_scripts/toggle-xkbmap.sh
cp $ROOT_DIR/resources/vanilla_origin_wide.png ~/Pictures/vanilla_origin_wide.png
cp $ROOT_DIR/resources/vanilla_origin_wide_double.png ~/Pictures/vanilla_origin_wide_double.png
