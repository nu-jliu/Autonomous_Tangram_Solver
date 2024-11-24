from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "puzzle_solver"


def generate_launch_description():
    arg_live = DeclareLaunchArgument(
        name="live",
        default_value="true",
        choices=["true", "false"],
        description="If use live model prediction or use pre-saved the snapshot",
    )

    node_publisher = Node(
        package=package_name,
        executable="image_publisher",
        name="image_publisher",
        parameters=[
            {
                "image_dir": PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
                        "images",
                    ]
                ),
                "image_name": "test.png",
            }
        ],
        condition=UnlessCondition(LaunchConfiguration("live")),
    )

    package_segment = "image_segmentation"

    node_segment = Node(
        package=package_segment,
        executable="puzzle_segment",
        # name="tangram_segment",
        parameters=[
            {
                "model_dir": PathJoinSubstitution(
                    [
                        FindPackageShare(package_segment),
                        "model",
                    ]
                ),
                "model_filename": "tangram_cae.pth",
                "image_dir": PathJoinSubstitution(
                    [
                        FindPackageShare(package_segment),
                        "images",
                    ]
                ),
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("live")),
    )

    node_detection = Node(
        package=package_name,
        executable=package_name,
        name=package_name,
        output="screen",
    )

    node_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
    )

    return LaunchDescription(
        [
            arg_live,
            node_publisher,
            node_segment,
            node_detection,
            node_video_server,
        ]
    )
