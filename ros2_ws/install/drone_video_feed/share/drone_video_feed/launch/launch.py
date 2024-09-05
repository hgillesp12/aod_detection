from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Path to the Gazebo ROS package
    gazebo_ros_path = '/Users/hannahgillespie/miniconda3/envs/ros_env/share/gazebo_ros'

    # World file path
    world_file = '/Users/hannahgillespie/aod_detection/ros2_ws/src/drone_video_feed/worlds/latest.world'

    # Model files paths
    quadcopter_model_file = '/Users/hannahgillespie/aod_detection/ros2_ws/src/drone_video_feed/models/quadcopter_with_camera/model.sdf'
    tree_model_file = '/Users/hannahgillespie/aod_detection/ros2_ws/src/drone_video_feed/models/blender_tree/model.sdf'

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

        # Spawn the quadcopter drone with a camera in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'quadcopter_with_camera',
                '-x', '0', '-y', '0', '-z', '1',  
                '-file', quadcopter_model_file
            ],
            output='screen'
        ),

        # Spawn the tree model in Gazebo
       Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-entity', 'blender_tree',
               '-x', '10', '-y', '0', '-z', '0',  
               '-file', tree_model_file
           ],
           output='screen'
       ),

        # Video Feed Node to process the camera feed
        # Node(
        #     package='drone_video_feed',
        #     executable='video_feed_node',
        #     name='video_feed_node',
        #     output='screen'
        # ),

        # RViz Node to visualize the processed camera feed
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/Users/hannahgillespie/aod_detection/ros2_ws/src/drone_video_feed/config/quadcopter_camera.rviz'],
        ),
    ])
