#!/bin/bash

# sleep for 5 seconds
sleep 5

# get the current path
current_dir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# cd to the current dir
cd $current_dir

# run the program
/usr/bin/python  main.py
