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
    # include_realsense = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("realsense2_camera"),
    #                 "launch",
    #                 "rs_launch.py",
    #             ]
    #         )
    #     ),
    #     launch_arguments={
    #         "camera_namespace": "puzzle",
    #         "serial_no": "_141722071471",
    #         "publish_tf": "false",
    #     }.items(),
    # )

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

    # include_scanner = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("image_segmentation"),
    #                 "launch",
    #                 "detect.launch.py",
    #             ]
    #         )
    #     ),
    #     launch_arguments={
    #         "stream": "false",
    #     }.items(),
    # )

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
        launch_arguments={
            "stream": "false",
            "use_rviz": "false",
        }.items(),
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

    include_calibration = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hand_eye_calibration"),
                    "launch",
                    "calibrate.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_rviz": "false",
            "launch_rs": "false",
            "launch_robot": "false",
        }.items(),
    )

    node_generator = Node(
        package=package_name,
        executable="action_generator",
        name="action_generator",
    )

    node_executor = Node(
        package=package_name,
        executable="action_executor",
        name="action_executor",
        parameters=[
            # {
            #     "standoff": 0.12,
            #     "object": 0.06,
            # }
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "robot.yaml",
                ]
            )
        ],
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "tangram.rviz",
                ]
            ),
        ],
    )

    node_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
    )

    return LaunchDescription(
        [
            # include_realsense,
            include_solver,
            # include_scanner,
            include_detector,
            include_controller,
            include_calibration,
            node_generator,
            node_executor,
            node_rviz,
            node_video_server,
        ]
    )
