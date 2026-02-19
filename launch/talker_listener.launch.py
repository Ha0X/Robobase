from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    topic = LaunchConfiguration("topic")
    publish_hz = LaunchConfiguration("publish_hz")
    message_prefix = LaunchConfiguration("message_prefix")

    return LaunchDescription(
        [
            DeclareLaunchArgument("topic", default_value="chatter"),
            DeclareLaunchArgument("publish_hz", default_value="1.0"),
            DeclareLaunchArgument("message_prefix", default_value="Hello ROS 2!"),
            Node(
                package="my_ros2_examples",
                executable="talker",
                name="talker",
                output="screen",
                parameters=[
                    {
                        "topic": topic,
                        "publish_hz": ParameterValue(publish_hz, value_type=float),
                        "message_prefix": message_prefix,
                    }
                ],
            ),
            Node(
                package="my_ros2_examples",
                executable="listener",
                name="listener",
                output="screen",
                parameters=[{"topic": topic}],
            ),
        ]
    )


