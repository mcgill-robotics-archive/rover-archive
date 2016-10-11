#!/bin/bash
#
# Initialize rover repository into working state


# Fail on first error
set -e

# Update repository
echo "Pulling latest changes..."
git pull
echo

# urg_node
echo "Installing urg_node..."
sudo apt-get install -y ros-kinetic-urg-node
echo

# convex_decompositon
echo "Installing convex_decomposition..."
sudo apt-get install -y ros-kinetic-convex-decomposition
echo

