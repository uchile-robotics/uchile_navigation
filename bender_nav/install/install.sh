#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
pkg_name="bender_nav";
install_path=$(rospack find $pkg_name)/install;
install_files=$install_path/files;
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# - - - - - - - Install dependencies - - - - - - - -
echo -e "\n$installer Installing bender_nav dependencies . . .\n"

bash $(rospack find bender_sensors)/install/install.sh


# - - - - - - - - - ROS packages  - - - - - - - - - - -
echo -e "\n$installer Installing navigation packages . . .\n"
sudo apt-get install ros-indigo-common-msgs ;
sudo apt-get install ros-indigo-slam-gmapping ;
sudo apt-get install ros-indigo-geometry ;
sudo apt-get install ros-indigo-navigation ;


#  - - - - - - - - - Build Navigation - - - - - - - - - - 
echo -e "\n$installer Building bender_navigation\n";
cd $BENDER_WORKSPACE/..
catkin_make --only-pkg-with-deps bender_nav

# :)
