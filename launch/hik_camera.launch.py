from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():
    package_share = get_package_share_directory("hik_camera_driver")
    default_params = os.path.join(package_share, "config", "camera_params.yaml")

    arguments = [
        DeclareLaunchArgument("camera_namespace", default_value=""),
        DeclareLaunchArgument("node_name", default_value="camera"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("transport", default_value="usb"),
        DeclareLaunchArgument("serial_number", default_value=""),
        DeclareLaunchArgument("user_defined_name", default_value=""),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument("frame_id", default_value="camera_optical_frame"),
        DeclareLaunchArgument("output_encoding", default_value="rgb8"),
        DeclareLaunchArgument("use_sensor_data_qos", default_value="true"),
        DeclareLaunchArgument("trigger_mode", default_value="off"),
    ]

    camera = Node(
        package="hik_camera_driver",
        executable="hik_camera_driver_node",
        namespace=LaunchConfiguration("camera_namespace"),
        name=LaunchConfiguration("node_name"),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "transport": ParameterValue(
                    LaunchConfiguration("transport"), value_type=str
                ),
                "serial_number": ParameterValue(
                    LaunchConfiguration("serial_number"), value_type=str
                ),
                "user_defined_name": ParameterValue(
                    LaunchConfiguration("user_defined_name"), value_type=str
                ),
                "camera_name": ParameterValue(
                    LaunchConfiguration("camera_name"), value_type=str
                ),
                "camera_info_url": ParameterValue(
                    LaunchConfiguration("camera_info_url"), value_type=str
                ),
                "frame_id": ParameterValue(
                    LaunchConfiguration("frame_id"), value_type=str
                ),
                "output_encoding": ParameterValue(
                    LaunchConfiguration("output_encoding"), value_type=str
                ),
                "use_sensor_data_qos": ParameterValue(
                    LaunchConfiguration("use_sensor_data_qos"), value_type=bool
                ),
                "trigger_mode": ParameterValue(
                    LaunchConfiguration("trigger_mode"), value_type=str
                ),
            },
        ],
    )

    return LaunchDescription(arguments + [camera])
