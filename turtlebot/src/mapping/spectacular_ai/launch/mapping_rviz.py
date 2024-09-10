from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz").perform(context)),
        package='rviz2', executable='rviz2', output='screen',
        arguments=['--display-config', 'launch/mapping.rviz'])

    return [
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value='True'),
            DeclareLaunchArgument("recordingFolder", default_value='')
        ] + [
            OpaqueFunction(function=launch_setup)
        ]
    )
