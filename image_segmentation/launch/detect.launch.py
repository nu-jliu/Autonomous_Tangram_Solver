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

    node_webcam = Node(
        package=package_name,
        executable="webcam",
        name="webcam",
        parameters=[{"camera_name": "Amcrest AWC2198 USB Webcam"}],
    )

    node_detect = Node(
        package=package_name,
        executable="shape_detect",
        name="shape_detect",
        parameters=[
            {
                "model_dir": PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
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

    node_stream = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        condition=IfCondition(LaunchConfiguration("stream")),
    )

    return LaunchDescription(
        [
            arg_stream,
            # include_realsense,
            node_webcam,
            node_detect,
            node_stream,
        ]
    )
