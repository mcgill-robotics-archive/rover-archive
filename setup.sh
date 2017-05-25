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
if [[ ! -h /etc/udev/rules.d/11-rover-udev.rules ]]; then
    echo "Setting up udev rules..."
    sudo ln -s ${ROBOTIC_PATH}/rover/11-rover-udev.rules /etc/udev/rules.d
fi
echo

# convex_decompositon
echo "Installing convex_decomposition..."
sudo apt-get install -qq ros-kinetic-convex-decomposition
echo

# urg_node
echo "Installing urg_node.."
sudo apt-get install -qq ros-kinetic-urg-node 
echo

# ROS dependency update
echo "Installing ros dependencies for catkin workspace..."
rosdep update
rosdep install -y --from-paths ./catkin_ws --ignore-src \
    --skip-keys=rover_msgs --skip-keys=ros_lib --rosdistro kinetic
echo

# Set up MoveIt!
echo "Installing MoveIt! from repositories..."
sudo apt-get install -qq ros-kinetic-moveit
echo

echo "Installing gcc-arm-none-eabi..."
sudo apt-get install -qq ros-kinetic-rosserial-tivac gcc-arm-none-eabi
echo

if [[ ! -d ../tivaware ]]; then
    echo "Setting up tivaware..."
    cd ..
    git clone git@github.com:mcgill-robotics/tivaware
    cd tivaware
    make clean
    make
    cd ../rover
    echo
fi

if [[ ! -d ../tiva ]]; then
    echo "Setting up tiva catkin_ws..."
    cd ..
    mkdir -p tiva/catkin_ws/src
    cd tiva/catkin_ws/src
    git clone git@github.com:mcgill-robotics/rover_tiva
    cd rover_tiva
    if [[ ! -h /etc/udev/rules.d/71-rover-tiva.rules ]]; then
        sudo ln -s \
        ${ROBOTIC_PATH}/tiva/catkin_ws/src/rover_tiva/71-rover-tiva.rules \
        /etc/udev/rules.d/71-rover-tiva.rules
    fi
    cd ${ROBOTIC_PATH}/rover
    echo
fi

if [[ ! -f /usr/local/bin/lm4flash ]]; then
    echo "Setting up lm4flash..."
    sudo cp tools/lm4flash /usr/local/bin/
    echo
fi

echo "All done with Rover setup!"
