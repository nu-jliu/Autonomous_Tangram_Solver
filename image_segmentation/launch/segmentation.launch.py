from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "image_segmentation"


def generate_launch_description():
    # arg_live = DeclareLaunchArgument(
    #     name="live",
    #     default_value="true",
    #     choices=["true", "false"],
    #     description="If use live model prediction or use pre-saved the snapshot",
    # )

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
    )

    return LaunchDescription(
        [
            node_segment,
            node_video_server,
        ]
    )
