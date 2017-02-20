#!/bin/bash
## Usage: bash setup1.sh
## Author: sumanth
## Purpose: clones the required vigir repos and compiles for 3d foot steps planning
##
## Options:
##   none
##


#check if the workspace is already set
# setup the workspace
if [ -d $"/home/$USER/catkin_ws" ]; then  
  WORKSPACE="catkin_ws"
  echo "$(tput setaf 1)found workspace catkin_ws$(tput sgr0)"
elif [ -d $"/home/$USER/indigo_ws" ]; then  
  WORKSPACE="indigo_ws"
  echo "$(tput setaf 1)found workspace indigo_ws$(tput sgr0)"  
else
  "$(tput setaf 1)no workspace found exiting$(tput sgr0)" 
  exit
fi

# create a folder and clone the required vigir libraries
mkdir /home/$USER/$WORKSPACE/src/vigir_footstepPlanning
# these repos are needed http://wiki.ros.org/vigir_footstep_planning
cd /home/$USER/$WORKSPACE/src/vigir_footstepPlanning
git clone https://github.com/team-vigir/vigir_footstep_planning_msgs.git
git clone https://github.com/team-vigir/vigir_footstep_planning_basics.git
git clone https://github.com/team-vigir/vigir_footstep_planning_core.git
git clone https://github.com/team-vigir/vigir_terrain_classifier.git
git clone https://github.com/team-vigir/vigir_generic_params.git
git clone https://github.com/team-vigir/vigir_pluginlib.git

# compile the code
cd ~/$WORKSPACE     
catkin_make


