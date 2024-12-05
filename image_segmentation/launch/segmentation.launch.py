from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "image_segmentation"


def generate_launch_description():
    arg_stream = DeclareLaunchArgument(
        name="stream",
        default_value="true",
        choices=["true", "false"],
        description="Whether to stream the video",
    )

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
            "camera_namespace": "piece",
            "pointcloud.enable": "true",
            "align_depth.enable": "true",
            "serial_no": "_243522073414",
        }.items(),
    )

    node_segment = Node(
        package=package_name,
        executable="piece_segment",
        name="piece_segment",
        parameters=[
            {
                "model_dir": PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
                        "model",
                    ]
                ),
                "model_file": "sam2.1_hiera_large.pt",
                "model_cfg": "configs/sam2.1/sam2.1_hiera_l.yaml",
            }
        ],
        # condition=UnlessCondition(LaunchConfiguration("live")),
    )

    node_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        condition=IfCondition(LaunchConfiguration("stream")),
    )

    return LaunchDescription(
        [
            arg_stream,
            include_realsense,
            node_segment,
            node_video_server,
        ]
    )
