from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = "puzzle_solver"


def generate_launch_description():
    arg_real_time = DeclareLaunchArgument(
        name="real_time",
        default_value="true",
        choices=["true", "false"],
        description="Whether to solve the puzzle in real time",
    )
    arg_stream = DeclareLaunchArgument(
        name="stream",
        default_value="true",
        choices=["true", "false"],
        description="Whether to stream the all image topics",
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
        condition=UnlessCondition(LaunchConfiguration("real_time")),
    )

    package_segment = "image_segmentation"

    node_webcam = Node(
        package=package_name,
        executable="web_camera",
        name="web_camera",
        # parameters=[{"camera_name": "Amcrest AWC2198 USB Webcam"}],
        parameters=[{"camera_name": "BisonCam,NB Pro"}],
    )

    node_detect = Node(
        package=package_segment,
        executable="shape_detect",
        name="shape_detect",
        parameters=[
            {
                "model_dir": PathJoinSubstitution(
                    [
                        FindPackageShare(package_segment),
                        "model",
                    ]
                ),
                "model_name": "shapes.pt",
                # "model_cfg": "configs/sam2.1/sam2.1_hiera_l.yaml",
            }
        ],
        # output="screen",
        # condition=UnlessCondition(LaunchConfiguration("live")),
    )

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
        condition=IfCondition(LaunchConfiguration("real_time")),
    )

    node_solver = Node(
        package=package_name,
        executable=package_name,
        name=package_name,
        output="screen",
    )

    node_pixel_to_real = Node(
        package=package_name,
        executable="solution_pixel_to_real",
        name="solution_pixel_to_real",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "env.yaml",
                ]
            )
        ],
    )

    node_generator = Node(
        package=package_name,
        executable="svg_generator",
        name="svg_generator",
    )

    node_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        condition=IfCondition(LaunchConfiguration("stream")),
    )

    return LaunchDescription(
        [
            arg_real_time,
            arg_stream,
            node_publisher,
            node_webcam,
            node_detect,
            node_segment,
            node_solver,
            node_pixel_to_real,
            node_generator,
            node_video_server,
        ]
    )
