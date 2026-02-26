from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    service_name = LaunchConfiguration("service_name")
    a = LaunchConfiguration("a")
    b = LaunchConfiguration("b")

    return LaunchDescription(
        [
            DeclareLaunchArgument("service_name", default_value="add_two_ints"),
            DeclareLaunchArgument("a", default_value="10"),
            DeclareLaunchArgument("b", default_value="20"),
            Node(
                package="my_ros2_examples",
                executable="add_server",
                name="add_server",
                output="screen",
                parameters=[{"service_name": service_name}],
            ),
            Node(
                package="my_ros2_examples",
                executable="add_client",
                name="add_client",
                output="screen",
                parameters=[
                    {
                        "service_name": service_name,
                        "a": ParameterValue(a, value_type=int),
                        "b": ParameterValue(b, value_type=int),
                    }
                ],
            ),
        ]
    )


