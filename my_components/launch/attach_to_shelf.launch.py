
import launch
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='pre_approach'),
                ComposableNode(
                    package='my_components',
                    plugin='my_components::AttachServer',
                    name='attach_server'),
            ],
            output='screen',
    )

    manual_composition_node = Node(
            package='my_components',
            executable='manual_composition',
            output='screen',
            emulate_tty=True)



    return launch.LaunchDescription([
            container, 
    ])