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

package_name = "tangram_bot"


def generate_launch_description():

    include_solver = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("puzzle_solver"),
                    "launch",
                    "puzzle_solver.launch.py",
                ]
            )
        ),
        launch_arguments={"stream": "false"}.items(),
    )

    include_detector = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("piece_detection"),
                    "launch",
                    "piece_detection.launch.py",
                ]
            )
        ),
        launch_arguments={"stream": "false"}.items(),
    )

    include_controller = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("maxarm_control"),
                    "launch",
                    "robot.launch.py",
                ]
            )
        ),
        # launch_arguments={"stream": "false"}.items(),
    )

    node_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
    )

    return LaunchDescription(
        [
            include_solver,
            include_detector,
            include_controller,
            node_video_server,
        ]
    )
