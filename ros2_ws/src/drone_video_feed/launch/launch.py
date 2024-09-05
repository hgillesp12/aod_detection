from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Path to the Gazebo ROS package
    gazebo_ros_path = '/Users/hannahgillespie/miniconda3/envs/ros_env/share/gazebo_ros'

    # World file path
    world_file = '/Users/hannahgillespie/ros2_ws/src/drone_video_feed/worlds/latest.world'

    return LaunchDescription([
        # Declare the 'world' argument
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='SDF world file'
        ),

        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': LaunchConfiguration('world')}.items()
        ),

        # Spawn the quadcopter drone with a camera
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'quadcopter_with_camera',
                '-x', '0', '-y', '0', '-z', '1',  # Initial position of the drone
                '-file', '/Users/hannahgillespie/ros2_ws/src/drone_video_feed/models/quadcopter_with_camera/model.sdf'  # Path to the quadcopter SDF file
            ],
            output='screen'
        ),

        # Controller Node for controlling the quadcopter with ROS 2 commands
        Node(
            package='drone_video_feed',  # Replace with the package name containing your control node
            executable='quadcopter_controller',  # Replace with the executable name of your control node
            output='screen',
            parameters=[
                {'controller_frequency': 30.0},  # Example parameter; adjust as needed
            ],
        ),

        # RViz Node to visualize the quadcopter's camera feed
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/Users/hannahgillespie/ros2_ws/src/drone_video_feed/config/quadcopter_camera.rviz'],  # Path to your RViz configuration file
        ),
    ]) 