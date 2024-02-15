import launch
import launch.actions
import launch.substitutions
import launch_ros.actions



def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[
                launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='my_package', 
            node_executable='my_publisher', 
            output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'my_publisher_node'],
            # parameters=[
            #     {"my_param_1": "param_1"},
            #     {"my_param_2": "param_2"},
            #     {"my_param_3": "param_3"}
            # ]
            ),
        launch_ros.actions.Node(
            package='my_package', node_executable='my_subscriber', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'my_subscriber_node']),
    ])
