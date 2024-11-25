from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "piece_detection"


def generate_launch_description():
    # arg_live = DeclareLaunchArgument(
    #     name="live",
    #     default_value="true",
    #     choices=["true", "false"],
    #     description="If use live model prediction or use pre-saved the snapshot",
    # )
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
            "json_file_path": PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "realsense_config.json",
                ]
            ),
        }.items(),
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

    node_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
    )

    return LaunchDescription(
        [
            include_realsense,
            node_segment,
            node_detection,
            node_video_server,
        ]
    )
