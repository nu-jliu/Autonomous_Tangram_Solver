from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    Command,
    TextSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage

package_name = "maxarm_control"


def generate_launch_description():
    node_align = Node(
        package=package_name,
        executable="frame_align",
        name="frame_align",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "world.yaml",
                ]
            )
        ],
        # remappings=[
        #     ("image_rect", "camera/camera/color/image_raw"),
        #     ("camera_info", "camera/camera/color/camera_info"),
        # ],
    )

    node_control = Node(
        package=package_name,
        executable="robot_control",
        name="robot_control",
        # arguments=[
        #     "-d",
        #     PathJoinSubstitution(
        #         [
        #             FindPackageShare(package_name),
        #             "config",
        #             "calibrate.rviz",
        #         ]
        #     ),
        # ],
        # condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            node_control,
            node_align,
        ]
    )
