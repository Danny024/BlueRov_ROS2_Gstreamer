# video_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bluerov_driver',  # The name of your package
            executable='video',        # The executable name (from add_executable in CMakeLists.txt)
            name='bluerov2_video_node',
            output='screen',
            parameters=[
                {'port': 5600},  # Set the UDP port
                {'topic': '/bluerov2/camera/compressed'}  # Set the ROS topic for publishing
            ]
        )
    ])
