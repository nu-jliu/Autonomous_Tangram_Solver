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

package_name = "piece_detection"


def generate_launch_description():
    arg_stream = DeclareLaunchArgument(
        name="stream",
        default_value="true",
        choices=["true", "false"],
        description="Whether to stream all video sources",
    )
    arg_use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Whether to use rviz to visualize",
    )

    package_segment = "image_segmentation"

    include_realsense = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ]
            )
        ),
        launch_arguments={
            "pointcloud.enable": "true",
            "align_depth.enable": "true",
            "serial_no": "_243522073414",
        }.items(),
    )

    node_rs_model = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="realsense_rsp",
        parameters=[
            {
                "robot_description": Command(
                    [
                        ExecutableInPackage("xacro", "xacro"),
                        TextSubstitution(text=" "),
                        PathJoinSubstitution(
                            [
                                FindPackageShare(package="realsense2_description"),
                                "urdf",
                                "test_d435i_camera.urdf.xacro",
                            ]
                        ),
                        TextSubstitution(text=" add_plug:=true"),
                        TextSubstitution(text=" use_nominal_extrinsics:=true"),
                    ]
                )
            }
        ],
    )

    node_segment = Node(
        package=package_segment,
        executable="piece_segment",
        name="piece_segment",
        parameters=[
            {
                "model_dir": PathJoinSubstitution(
                    [
                        FindPackageShare(package_segment),
                        "model",
                    ]
                ),
                "model_file": "sam2.1_hiera_large.pt",
                "model_cfg": "configs/sam2.1/sam2.1_hiera_l.yaml",
            }
        ],
        # condition=UnlessCondition(LaunchConfiguration("live")),
    )

    node_detection = Node(
        package=package_name,
        executable="piece_detection",
        name="piece_detection",
    )

    node_pixel_to_real = Node(
        package=package_name,
        executable="rs_pixel_to_real",
        name="rs_pixel_to_real",
    )

    node_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        condition=IfCondition(LaunchConfiguration("stream")),
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
                    "detection.rviz",
                ]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            arg_stream,
            arg_use_rviz,
            include_realsense,
            node_rs_model,
            node_segment,
            node_detection,
            node_pixel_to_real,
            node_video_server,
            node_rviz,
        ]
    )
