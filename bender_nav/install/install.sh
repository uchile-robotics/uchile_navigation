#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# - - - - - - - Install dependencies - - - - - - - -
echo -e "\n$installer Installing bender_nav dependencies . . .\n"

bender_cd  bender_sensors
bash install/install.sh


# - - - - - - - - - ROS packages  - - - - - - - - - - -
echo -e "\n$installer Installing navigation packages . . .\n"
sudo apt-get install ros-indigo-common-msgs 
sudo apt-get install ros-indigo-slam-gmapping 
sudo apt-get install ros-indigo-geometry 
sudo apt-get install ros-indigo-navigation 


# :)
