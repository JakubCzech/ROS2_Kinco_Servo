from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    port = LaunchConfiguration("port", default="/dev/ttyUSB0")
    baudrate = LaunchConfiguration("baudrate", default=115200)
    timeout = LaunchConfiguration("timeout", default=0.1)
    target_high_topic = LaunchConfiguration("target_high_topic", default="target_high")
    state_topic = LaunchConfiguration("state_topic", default="state")
    namespace = LaunchConfiguration("namespace", default="")

    declare_port = DeclareLaunchArgument(
        "port",
        default_value=port,
        description="The port to connect to",
    )
    declare_baudrate = DeclareLaunchArgument(
        "baudrate",
        default_value=baudrate,
        description="The baudrate to connect with",
    )
    declare_timeout = DeclareLaunchArgument(
        "timeout",
        default_value=timeout,
        description="The timeout to connect with",
    )
    declare_target_high_topic = DeclareLaunchArgument(
        "target_high_topic",
        default_value=target_high_topic,
        description="The topic to publish target_high",
    )
    declare_state_topic = DeclareLaunchArgument(
        "state_topic",
        default_value=state_topic,
        description="The topic to publish state",
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value=namespace,
        description="The namespace to run in",
    )

    return LaunchDescription(
        [
            declare_port,
            declare_baudrate,
            declare_timeout,
            declare_target_high_topic,
            declare_state_topic,
            declare_namespace,
            Node(
                package="kinco_driver",
                executable="kinco_driver_node",
                name="kinco_driver_node",
                namespace=namespace,
                parameters=[
                    {"port": port},
                    {"baudrate": baudrate},
                    {"timeout": timeout},
                    {"target_high_topic": target_high_topic},
                    {"state_topic": state_topic},
                ],
            ),
        ]
    )
