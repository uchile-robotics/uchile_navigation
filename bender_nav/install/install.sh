#!/bin/bash
#
# Run me like this
# > bash install.sh
#
# DO NOT USE ONE OF THIS:
# > source install.sh
# > . install.sh
# > ./install.sh
#

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #
source "$BENDER_WS"/bender_system/install/pkg_install.bash

# Useful Variables
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# - - - - - - - Install dependencies - - - - - - - -
echo -e "\n$installer Installing bender_nav dependencies . . .\n"

bender_cd bender_sensors
bash install/install.sh


# - - - - - - - - - ROS packages  - - - - - - - - - - -
echo -e "\n$installer Installing navigation packages . . .\n"
sudo apt-get install ros-indigo-common-msgs 
sudo apt-get install ros-indigo-slam-gmapping 
sudo apt-get install ros-indigo-geometry 
sudo apt-get install ros-indigo-navigation 


# :)
