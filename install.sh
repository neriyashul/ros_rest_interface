#!/bin/bash

# Check if an path is provided
if [ $# -eq 0 ]; then
     path=~/catkin_ws
else
    path="$1"
fi

# Use the path in further operations
echo "Using path: $path"


echo "copy directory 'ros_rest_interface' to "''$path/src''
cp -rp "ros_rest_interface/" $path/src



echo "Installing Casablanca (cpprestsdk)"
sudo apt-get install libcpprest-dev

cd $path
source devel/setup.bash
catkin_make