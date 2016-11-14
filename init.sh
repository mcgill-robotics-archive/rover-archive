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

# Ask if they want to install MoveIt!
echo "The following is mandatory for arm development."
echo "The initial build takes ~30 minutes."
ask="${question} Do you want to install MoveIt?${reset}"
read -p "${ask} [y/N] (default no) " moveit
case "${moveit}" in
  y|Y )
    echo "MoveIt will be installed!"
    export INSTALL_MOVEIT=true
    ;;
  * )
    echo "Ok np."
    echo "You can simply rerun this initialize script if you change your mind."
    export INSTALL_MOVEIT=false;
    ;;
esac

# Update from ubuntu repositories
echo "Updating and upgrading using apt..."
sudo apt-get update
sudo apt-get dist-upgrade
echo

# Update repository
echo "Pulling latest changes..."
git pull && git submodule update --init --recursive
echo

# convex_decompositon
echo "Installing convex_decomposition..."
sudo apt-get install -y ros-kinetic-convex-decomposition
echo

# ROS dependency update
echo "Installing ros dependencies for catkin workspace..."
rosdep update
rosdep install --from-paths ./catkin_ws --ignore-src --skip-keys=rover_msgs \
    --rosdistro kinetic
echo

# Build the catkin_ws
echo "Building the catkin_ws..."
pushd catkin_ws
catkin_make
popd
echo

# Set up MoveIt! if specified
if [[ ${INSTALL_MOVEIT} ]]; then
  echo "${section}MoveIt! Installation${reset}"
  
  # Install wstool and catkin-tools
  echo "Installing catkin build tool..."
  sudo apt-get install python-wstool python-catkin-tools
  echo

  # Go to the src directory of the workspace
  pushd moveit/
  pushd src/

  # Initialize ws
  echo "Initializing MoveIt! Workspace..."
  wstool init .
  wstool merge https://raw.githubusercontent.com/ros-planning/moveit/kinetic-devel/moveit.rosinstall
  wstool update
  echo

  # Install dependencies
  echo "Installing dependencies for MoveIt workspace..."
  rosdep install --from-paths . --ignore-src --rosdistro kinetic
  echo
  
  # Move into moveit workspace for build step
  popd

  # Build the workspace
  echo "Building workspace, this will take a while..."
  catkin config --extend /opt/ros/kinetic --cmake-args \
      -DCMAKE_BUILD_TYPE=Release
  catkin build
  echo

  # Back to workspace root dir
  popd
fi

echo "All done with Rover setup!"
