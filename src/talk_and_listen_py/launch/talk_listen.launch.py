from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='minimal_talker_py',
            executable='minimal_talker',
            name='minimal_talker'
        ),
        Node(
            package='minimal_listener_py',
            executable='minimal_listener',
            name='minimal_listener'
        ),
   ]) 
