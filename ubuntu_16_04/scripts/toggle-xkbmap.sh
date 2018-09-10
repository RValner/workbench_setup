#!/bin/bash
#
# Quickly switch between a given keyboard layout and the US Qwerty one

OTHER_LAYOUT="ee"
CURRENT="$(setxkbmap -query | grep layout | perl -pe 's/^layout: +([^ ]+)$/$1/')"

if [ "$CURRENT" = "us" ] ; then
    setxkbmap -layout "$OTHER_LAYOUT" -option ctrl:nocaps
    notify-send "Layout changed to" "Estonian" -t 1000 --urgency=low
else
    setxkbmap -layout us -option ctrl:nocaps
    notify-send "Layout changed to" "US" -t 1000 --urgency=low
fi
