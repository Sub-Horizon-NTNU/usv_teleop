from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Runs ON THE BOAT, next to the Raspberry Pi. Subscribes selene/light_state
    # (published by light_state_publisher on the controller/PC side) and drives
    # the Pi over local UDP. Fails safe (red, relay open) if the topic goes stale.
    pi_ip_arg = DeclareLaunchArgument(
        "pi_ip", default_value="192.168.2.5",
        description="Raspberry Pi address on the boat LAN")

    relay_driver = Node(
        package="usv_teleop",
        executable="light_relay_driver",
        name="light_relay_driver",
        output="screen",
        parameters=[{"pi_ip": LaunchConfiguration("pi_ip")}],
    )

    return LaunchDescription([pi_ip_arg, relay_driver])
