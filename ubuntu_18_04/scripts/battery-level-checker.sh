#!/bin/bash

# Allows to show "notify-send" notifications when executed under root user (cron)
export DISPLAY=:0

# State of the power module (chargigng/discharging)
STATE=`upower -i $(upower -e | grep 'BAT') | grep state: | awk '{print $2}'`

# Get the charge level where the battery is considered to be full
EFFECTIVE_MAX_LEVEL=`upower -i $(upower -e | grep 'BAT') | grep energy-full: | awk '{print $2}'`

# Get the current charge
CURRENT_LEVEL=`upower -i $(upower -e | grep 'BAT') | grep energy: | awk '{print $2}'`

# Control the battery only if it is discharging
if [ "$STATE" == "discharging" ] ; then
  
  # Calculate the battery level in %
  PERCENTAGE=$(bc <<<"scale=2;100*$CURRENT_LEVEL/$EFFECTIVE_MAX_LEVEL")
  PERCENTAGE=${PERCENTAGE%.*} # Converting it into integer. Dunno how to do a one liner, not feeling like googling

  # Battery levels (%)
  NORMAL=50     # Starts showing notifications
  LOW=20        # Still shows notifications but has more red into it
  CRITICAL=10   # Notifications become bigger and meaner
  SLEEP=6       # System goes to sleep (hibernate is preferred but my ubuntu 16.04 cannot hibernate)

  if [ "$PERCENTAGE" -le "$NORMAL" ] && [ "$PERCENTAGE" -gt "$LOW" ] ; then
    sudo -u robert /usr/bin/notify-send "Battery at $PERCENTAGE%" -t 1 --urgency=normal

  elif [ "$PERCENTAGE" -le "$LOW" ] && [ "$PERCENTAGE" -gt "$CRITICAL" ] ; then
    sudo -u robert /usr/bin/notify-send "Battery at $PERCENTAGE% \nPlug in the charger" -t 1 --urgency=critical

  elif [ "$PERCENTAGE" -le "$CRITICAL" ] && [ "$PERCENTAGE" -gt "$SLEEP" ] ; then
    sudo -u robert /usr/bin/notify-send "Battery at $PERCENTAGE% \nPlug in the charger! \n\n\n\n\n\n\n\nWill go to sleep on 6%!" -t 1 --urgency=critical

  elif [ "$PERCENTAGE" -le "$SLEEP" ] ; then
    sudo -u robert /usr/bin/notify-send "Battery at $PERCENTAGE% \nGOOD NIGHT" -t 1 --urgency=critical
    sudo -u root pm-suspend

  fi
fi
