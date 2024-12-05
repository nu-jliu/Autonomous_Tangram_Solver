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

package_name = "hand_eye_calibration"


def generate_launch_description():
    arg_use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Whether to use rviz to visualize",
    )

    arg_launch_rs = DeclareLaunchArgument(
        name="launch_rs",
        default_value="true",
        choices=["true", "false"],
        description="Whether launch realsense",
    )

    arg_launch_robot = DeclareLaunchArgument(
        name="launch_robot",
        default_value="true",
        choices=["true", "false"],
        description="Whether launch robot",
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
        condition=IfCondition(LaunchConfiguration("launch_rs")),
    )

    node_rs_model = Node(
        name="rs_model",
        package="robot_state_publisher",
        executable="robot_state_publisher",
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
        condition=IfCondition(LaunchConfiguration("launch_rs")),
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
        condition=IfCondition(LaunchConfiguration("launch_robot")),
        # launch_arguments={"stream": "false"}.items(),
    )

    node_apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "april_tags.yaml",
                ]
            )
        ],
        remappings=[
            ("image_rect", "piece/camera/color/image_raw"),
            ("camera_info", "piece/camera/color/camera_info"),
        ],
    )

    node_calibrate = Node(
        package=package_name,
        executable="calibrate",
        name="calibrate",
        parameters=[
            {
                "source_frame": "camera_color_optical_frame",
                "target_frame": "arm",
            }
        ],
    )

    node_scanner = Node(
        package=package_name,
        executable="scan_apriltag.py",
        name="scan_apriltag",
        # parameters=[
        #     {
        #         "source_frame": "camera_color_optical_frame",
        #         "target_frame": "arm",
        #     }
        # ],
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
                    "calibrate.rviz",
                ]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            arg_use_rviz,
            arg_launch_rs,
            arg_launch_robot,
            include_realsense,
            node_rs_model,
            include_controller,
            node_apriltag,
            node_calibrate,
            node_scanner,
            node_rviz,
        ]
    )
