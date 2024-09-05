#!/bin/bash

# Check for existing Conda environment and create if necessary
if conda info --envs | grep -q "ros_env"; then
  echo "ROS environment 'ros_env' already exists. Activating..."
  mamba activate ros_env
else
  echo "Creating and activating ROS environment..."
  mamba create -n ros_env python=3.10 -y
  mamba activate ros_env
fi

# Add channels
echo "Adding robostack channels..."
# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, this might return an error if it is not in the list which is ok
conda config --env --remove channels defaults

# Install ROS 2 Humble desktop
echo "Installing ROS 2 Humble desktop..."
mamba install ros-humble-desktop -y

# Install additional development tools
echo "Installing additional tools for development..."
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep -y

# Install Gazebo packages
echo "Installing Gazebo packages..."
mamba install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros -y

# Install additional Python packages
echo "Installing Python packages for ultralytics..."
pip install setuptools==70.0.0  # Install a version compatible with Gazebo and ultralytics
pip install numpy opencv-python pandas ultralytics

# Configure the environment setup script for ROS 2
echo "Setting up the environment script for ROS 2..."
cat << EOF > install/setup.bash
# Environment setup script generated for ROS 2 and Gazebo compatibility

# Source local setup script if exists
if [ -f "\$(dirname "\${BASH_SOURCE[0]}")/local_setup.bash" ]; then
  source "\$(dirname "\${BASH_SOURCE[0]}")/local_setup.bash"
fi

EOF

echo "Setup completed. Your ROS environment is ready."

# Verify final configuration
echo "Verifying ROS environment setup..."
echo "Active Conda Environment: $(conda info --envs | grep \*)"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# Test ROS 2 installation
ros2 topic list
