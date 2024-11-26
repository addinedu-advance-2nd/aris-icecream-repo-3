from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='bartendroid', 
        #     executable='master_for_test1118', 
        #     output='screen',
        # ),
        Node(
            package='bartendroid',
            executable='obj_detect_pub',
            output = 'screen'
        ),
        Node(
            package='bartendroid',
            executable='camera_server',
            output = 'screen'
        )
    ])