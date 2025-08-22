from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Start the keyboard node
        Node(
            package='mirte_workshop',       # package name
            executable='mirte_keyboard.py', # console_scripts name from setup.py
            name='mirte_keyboard',          # optional ROS node name
            output='screen'                 # show its log in the terminal
        )
        # To start other nodes, copy the "Node" block and replace the file name and ROS node name
    ])
