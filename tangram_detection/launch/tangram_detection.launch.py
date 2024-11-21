from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

package_name = "tangram_detection"


def generate_launch_description():
    node_publisher = Node(
        package=package_name,
        executable="image_publisher",
        name="image_publisher",
        parameters=[
            {
                "image_dir": "/home/jingkun/Documents/Final_Project/src/Autonomous_Tangram_Solver/model/test",
                "image_name": "output_image_cae.png",
            }
        ],
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
            node_publisher,
            node_detection,
            node_video_server,
        ]
    )
