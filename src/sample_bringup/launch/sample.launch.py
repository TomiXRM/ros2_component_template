from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    component_node_a = ComposableNode(
        package='component_template',
        plugin='component_template::ComponentTemplate',
        name='component_template_node_A',
        namespace='sample',
        parameters=[
            {
                "initial_count": 0,
                "topic_name": "example_int_A",
                "interval_ms": 100,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    component_node_b = ComposableNode(
        package='component_template',
        plugin='component_template::ComponentTemplate',
        name='component_template_node_B',
        namespace='sample',
        parameters=[
            {
                "initial_count": 1000,
                "topic_name": "example_int_B",
                "interval_ms": 1000,
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
            component_node_a,
            component_node_b,
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([container])
