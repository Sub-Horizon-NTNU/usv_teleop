from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0',
        description='Joystick device node')

    # joy driver: autorepeat keeps Joy flowing while a button/axis is held so the
    # deadman + command stay fresh even with the stick stationary.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )

    teleop_node = Node(
        package='usv_teleop',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        teleop_node,
    ])
