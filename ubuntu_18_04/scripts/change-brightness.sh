#!/bin/bash

# This script uses xrandr to change the brightness of the display

SCREEN_NAME=$(xrandr | grep " connected" | cut -f1 -d" ")
INCREASE=$1

# Convert the brightness levels from 0-1 float to 0-100 int and perform the calculations with ints
CURRENT_BRIGHTNESS_F=$(xrandr --verbose | grep -m 1 -i brightness | cut -f2 -d ' ')
CURRENT_BRIGHTNESS_I=$(echo "$CURRENT_BRIGHTNESS_F*100" | bc -l)
CURRENT_BRIGHTNESS_I=${CURRENT_BRIGHTNESS_I%.*}
NEW_BRIGHTNESS_I=$(($CURRENT_BRIGHTNESS_I+$INCREASE))

if [ $NEW_BRIGHTNESS_I -gt 100 ]; then
    exit;
fi

if [ $NEW_BRIGHTNESS_I -lt 0 ]; then
    exit;
fi

# Convert back to float
NEW_BRIGHTNESS_F="$(( $NEW_BRIGHTNESS_I / 100)).$(( $NEW_BRIGHTNESS_I % 100 ))"
xrandr --output $SCREEN_NAME --brightness $NEW_BRIGHTNESS_F --auto;

