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

# navigation stack fork install
# TODO: unificar como metodo llamado desde bender_system 
# (esto se usa en el instalador de bender)
bender_cd forks
if [ ! -d navigation ]; then
  echo "Cloning -navigation- fork from github."
  git clone https://github.com/uchile-robotics/navigation.git
  cd navigation
  git checkout kinetic-devel
else
  echo "-navigation- fork already exists. updating"
  cd navigation
  git checkout -- .
  git fetch
  git checkout kinetic-devel
fi

# :)
