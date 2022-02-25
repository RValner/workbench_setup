#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

STR_OLD=$1
STR_NEW=$2
NAME_HINT=$3
SCRIPT_NAME=~/.custom_scripts/python/format_escaped_string.py

STR_OLD_ESC=$(python2.7 $SCRIPT_NAME "$STR_OLD")
STR_NEW_ESC=$(python2.7 $SCRIPT_NAME "$STR_NEW")

echo -e $GREEN$BOLD"\nString old:"$RESET $STR_OLD
echo -e $GREEN$BOLD"String new:"$RESET $STR_NEW
echo -e $GREEN$BOLD"Escaped string old:"$RESET $STR_OLD_ESC
echo -e $GREEN$BOLD"Escaped string new:"$RESET $STR_NEW_ESC

FILES=""

if [ -n "$NAME_HINT" ]; then
  echo -e $GREEN$BOLD"\nList of potential files:"$RESET
  find . -type f -name "*$NAME_HINT*" -print0 | xargs -0 grep -n --color=always --with-filename "$STR_OLD"
  FILES=$(find . -type f -name "*$NAME_HINT*" -print0 | xargs -0 grep --with-filename "$STR_OLD" | cut -d':' -f1)
else
  echo -e $GREEN$BOLD"\nList of potential files:"$RESET
  grep -R -n --color=always --with-filename "$STR_OLD"
  FILES=$(grep -R --with-filename "$STR_OLD" | cut -d':' -f1)
fi

FILES_UNIQ=$(printf "%s\n" "${FILES[@]}" | sort -u)

echo
read -p "Do you want to continue? (y/n) " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]
then
  for file_name in "${FILES_UNIQ[@]}"
  do
    sed -i "s/$STR_OLD_ESC/$STR_NEW_ESC/g" $file_name
  done

  echo "Done"
else
  echo "Cancelled"
fi
