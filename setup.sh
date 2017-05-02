#!/bin/bash
#
# Initialize rover repository into working state


# Fail on first error
set -e

# Colors <3
header=$(tput setab 1; tput setaf 0)
question=$(tput bold; tput setaf 4)
section=$(tput bold; tput setaf 2)
warning=$(tput bold; tput setaf 1)
reset=$(tput sgr0)

# Welcome the user
echo "${header} Welcome to the Rover init script." 
echo "This script automates initial dependency resolution and building of the"
echo "workspace. It also optionally handles the setup of MoveIt! You will" 
echo "likely be prompted several times within the first minute.${reset}"
echo

# Ask for sudo privileges if not already availlable.
sudo echo "Checking for sudo priviledges..." || sudo -K
sudo echo "Sudo priviledges are availlable!" || exit 1
echo

# Update from ubuntu repositories
echo "Updating and upgrading using apt..."
sudo apt-get update
echo

# Update repository
echo "Pulling latest changes..."
git pull && git submodule update --init --recursive
echo

# Link udev rules
echo "Setting up udev rules..."
sudo ln -s `pwd`/11-rover-udev.rules /etc/udev/rules.d || echo "Aleady set."
echo

# convex_decompositon
echo "Installing convex_decomposition..."
sudo apt-get install -y ros-kinetic-convex-decomposition
echo

# urg_node
echo "Installing urg_node.."
sudo apt-get install ros-kinetic-urg-node 
echo

# ROS dependency update
echo "Installing ros dependencies for catkin workspace..."
rosdep update
rosdep install --from-paths ./catkin_ws --ignore-src --skip-keys=rover_msgs \
    --skip-keys=ros_lib --rosdistro kinetic
echo

# Set up MoveIt!
echo "Installing MoveIt! from repositories..."
sudo apt-get install ros-kinetic-moveit
echo

echo "All done with Rover setup!"
