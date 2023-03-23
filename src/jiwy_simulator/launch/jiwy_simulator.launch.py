from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jiwy_simulator',
            executable='jiwy_simulator',
            name='jiwy_simulator'
        ),
        Node(
            package='light_position_indicator',
            executable='light_position_indicator',
            name='light_position_indicator'
        ),
        Node(
            package='image_tools',
            executable='cam2image',
            name='camera'
        ),
        Node(
            package='image_tools',
            executable='showimage',
            name='showimage'
        ),
        Node(
             package='minimal_subscriber',
             executable='minimal_subscriber',
             name='minimal_subscriber'
         ),
#        Node(
#             package='point_publisher',
#             executable='point_publisher',
#             name='point_publisher'
#         ),
    ])