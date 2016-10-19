#!/bin/bash
#
# Initialize rover repository into working state


# Fail on first error
set -e

# Update repository
echo "Pulling latest changes..."
git pull
echo

# convex_decompositon
echo "Installing convex_decomposition..."
sudo apt-get install -y ros-kinetic-convex-decomposition
echo

# ROS dependencies
rosdep install --from-paths ./catkin_ws --ignore-src --skip-keys=rover_msgs

