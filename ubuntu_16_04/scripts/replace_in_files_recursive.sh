#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

SEARCH_KEYWORD="$1"
STR_OLD=$2
STR_NEW=$3
SCRIPT_NAME=~/.customScripts/python/format_escaped_string.py

STR_OLD_ESC=($(python2.7 $SCRIPT_NAME $STR_OLD))
STR_NEW_ESC=($(python2.7 $SCRIPT_NAME $STR_NEW))
SEARCH_KEYWORD_ESC=($(python2.7 $SCRIPT_NAME $SEARCH_KEYWORD))

echo -e $GREEN$BOLD"\nSearch keyword:"$RESET $SEARCH_KEYWORD
echo -e $GREEN$BOLD"String old:"$RESET $STR_OLD
echo -e $GREEN$BOLD"String new:"$RESET $STR_NEW
echo -e $GREEN$BOLD"Escaped search keyword:"$RESET $SEARCH_KEYWORD_ESC
echo -e $GREEN$BOLD"Escaped string old:"$RESET $STR_OLD_ESC
echo -e $GREEN$BOLD"Escaped string new:"$RESET $STR_NEW_ESC

echo -e $GREEN$BOLD"\nList of potential files:"$RESET
FILES=($(find . -name '*'$SEARCH_KEYWORD_ESC))

for file_name in "${FILES[@]}"
do
  echo ' -' "$file_name"
done

echo
read -p "Do you want to continue? (y/n) " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
  $(find . -name '*'$SEARCH_KEYWORD_ESC -exec sed -i "s/$STR_OLD_ESC/$STR_NEW_ESC/g" {} +)
  echo "Done"
else
  echo "Cancelled"
fi