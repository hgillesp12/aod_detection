#!/bin/bash

# Initialize Conda for the current shell
echo "Initializing Conda for the current shell..."
conda init "$(basename "$SHELL")"

# Reload the shell to apply changes from conda init
echo "Reloading the shell to apply changes..."
source ~/.bashrc  # Adjust this for your shell, e.g., ~/.zshrc for zsh

# Create and activate ROS environment
echo "Creating and activating ROS environment..."
conda create -n ros_env python=3.10 -y
conda activate ros_env

# Add channels and turn off the default one
echo "Configuring conda channels..."
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults

# Install ROS 2 desktop
echo "Installing ROS 2 Humble desktop..."
conda install ros-humble-desktop -y

# Install additional development tools
echo "Installing additional tools for development..."
conda install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep -y

# Install Gazebo packages
echo "Installing Gazebo packages..."
conda install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros -y

# Install additional Python packages
echo "Installing Python packages for ultralytics..."
pip install setuptools==70.0.0  # Install a version compatible with Gazebo and ultralytics
pip install numpy opencv-python pandas ultralytics

# Configure the environment setup script for ROS 2
echo "Setting up the environment script for ROS 2..."
cat << EOF > install/setup.bash
# Environment setup script generated for ROS 2 and Gazebo compatibility
# This script extends the environment with the environment of other prefix
# paths which were sourced when this file was generated as well as all packages
# contained in this prefix path.

# function to source another script with conditional trace output
# first argument: the path of the script
_colcon_prefix_chain_bash_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo "# . \"$1\""
    fi
    . "$1"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}

# Optional: Adjust or comment out this section if sourcing from the Conda environment causes issues
# COLCON_CURRENT_PREFIX="/Users/hannahgillespie/miniconda3/envs/ros_env"
# _colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"

# Correctly source this prefix (workspace's install directory)
# setting COLCON_CURRENT_PREFIX avoids determining the prefix in the sourced script
COLCON_CURRENT_PREFIX="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"
_colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"

unset COLCON_CURRENT_PREFIX
unset _colcon_prefix_chain_bash_source_script
EOF

echo "Setup completed. Your ROS environment is ready."