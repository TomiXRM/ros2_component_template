from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    component_node0 = ComposableNode(
        package='component_template',
        plugin='component_template::ComponentTemplate',
        name='component_template_node',
        namespace='sample',
        parameters=[
            {
                "counter": 0,
                "topic_name": "example_int0"
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    component_node1 = ComposableNode(
        package='component_template',
        plugin='component_template::ComponentTemplate',
        name='component_template_node',
        namespace='sample',
        parameters=[
            {
                "counter": 1000,
                "topic_name": "example_int1"
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name='sample_component_container',
        namespace='sample',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            component_node0,
            component_node1,
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([container])
