# aod_detection
Thesis for detecting Acute Oak Decline (AOD) in oak trees across the UK using object detection. 

## Google Drive Link
Please access the Google Drive link for all other resources:
https://drive.google.com/drive/folders/1d1tS1PEgrRGv_fTEtfmvFfJ5abwboc1l?usp=sharing

`colab_notebooks` directory:
- `video_replay.ipynb`: applies model and runs inference on input video, saves processed videos in sample_videos directory
- `augment_data.ipynb`: runs augmentation techniques on images and saves back to images folder
- `y`olo_custom_model_training.ipynb`: builds custom yolov5 model based on yolov5s through transfer learning

## Create Local ROS2_ENV Environment 
Source: RoboStack (https://robostack.github.io/GettingStarted.html)
```bash
# create and activate ROS environment
conda create -n ros_env python=3.10
conda init
conda activate ros_env

# Add channels and turn off default one
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults

# Full ROS2 desktop install
conda install ros-humble-desktop

# Additional tools for local dev
conda install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep

# Install Gazebo packages
conda install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
```

### Reset setup.bash
The `install/setup.bash` script can be set:
```bash
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
```

## Run Simulation
```bash
conda activate ros2_env
source install/setup.bash
colcon build
ros2 launch drone_video_feed launch.py
```