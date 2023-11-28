import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('attach_shelf')

    obstacle_arg = DeclareLaunchArgument('obstacle', default_value="0.3")
    degrees_arg = DeclareLaunchArgument('degrees', default_value="-90")

    obstacle_ = LaunchConfiguration("obstacle")
    degrees_ = LaunchConfiguration("degrees")

    pre_approach_node = Node(
        package="attach_shelf",
        executable="pre_approach_v2_node",
        output="screen",
        name= "pre_approach_v2_node",
        parameters=[{"obstacle": obstacle_,
                    "degrees": degrees_,}]
    )

    rviz_config_dir = os.path.join(pkg_path, 'rviz', 'cp9.rviz')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output='screen',
        name="rviz_node",
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        pre_approach_node,
    ])
