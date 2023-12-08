import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
import launch


def generate_launch_description():

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(
        'my_components'), 'rviz', 'cp10.rviz')


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])


    test_node_action = Node(
        package='my_components',
        executable='pre_approach_component_node',
        output='screen',
        name ='pre_approach',
        emulate_tty=True,
    )

    # create and return launch description object
    return LaunchDescription(
        [
            rviz_node,
            test_node_action,
        ]
    )

