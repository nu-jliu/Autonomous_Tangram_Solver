from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

package_name = "tangram_detection"


def generate_launch_description():
    node_publisher = Node(
        package=package_name,
        executable="image_publisher",
        name="image_publisher",
        parameters=[
            {
                "image_dir": "/home/jingkun/Documents/Final_Project/src/Autonomous_Tangram_Solver/model/dataset/output_cae",
                "image_name": "21.png",
            }
        ],
    )

    node_segment = Node(
        package=package_name,
        executable="tangram_segment.py",
        # name="tangram_segment",
        parameters=[
            {
                "model_dir": PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
                        "model",
                    ]
                ),
                "model_filename": "tangram_cae.pth",
                "image_dir": PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
                        "images",
                    ]
                ),
            }
        ],
        output="screen",
    )

    node_detection = Node(
        package=package_name,
        executable="tangram_detection",
        name="tangram_detection",
    )

    node_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
    )

    return LaunchDescription(
        [
            # node_publisher,
            node_segment,
            node_detection,
            node_video_server,
        ]
    )
