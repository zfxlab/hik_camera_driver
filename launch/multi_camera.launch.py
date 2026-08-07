import os
import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_STRING_PARAMETERS = {
    "transport",
    "serial_number",
    "user_defined_name",
    "camera_name",
    "camera_info_url",
    "frame_id",
    "output_encoding",
    "trigger_mode",
}
_BOOL_PARAMETERS = {"use_sensor_data_qos", "auto_exposure", "auto_gain"}
_INTEGER_PARAMETERS = {
    "grab_timeout_ms",
    "reconnect_interval_ms",
    "max_consecutive_timeouts",
    "sdk_buffer_count",
}
_DOUBLE_PARAMETERS = {"exposure_time", "gain", "frame_rate"}
_DRIVER_PARAMETERS = (
    _STRING_PARAMETERS | _BOOL_PARAMETERS | _INTEGER_PARAMETERS | _DOUBLE_PARAMETERS
)
_LAUNCH_KEYS = {"enabled", "namespace", "node_name"}
_ROS_NAME_TOKEN_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _require_mapping(value, description):
    if not isinstance(value, dict):
        raise RuntimeError(f"{description} must be a YAML mapping")
    return value


def _validate_parameters(parameters, description):
    unknown = set(parameters) - _DRIVER_PARAMETERS
    if unknown:
        raise RuntimeError(
            f"{description} contains unknown driver parameters: {sorted(unknown)}"
        )

    normalized = dict(parameters)
    for key in _STRING_PARAMETERS & set(normalized):
        if not isinstance(normalized[key], str):
            hint = " (quote serial numbers in YAML)" if key == "serial_number" else ""
            raise RuntimeError(f"{description}.{key} must be a string{hint}")
    for key in _BOOL_PARAMETERS & set(normalized):
        if not isinstance(normalized[key], bool):
            raise RuntimeError(f"{description}.{key} must be true or false")
    for key in _INTEGER_PARAMETERS & set(normalized):
        if isinstance(normalized[key], bool) or not isinstance(normalized[key], int):
            raise RuntimeError(f"{description}.{key} must be an integer")
    for key in _DOUBLE_PARAMETERS & set(normalized):
        if isinstance(normalized[key], bool) or not isinstance(normalized[key], (int, float)):
            raise RuntimeError(f"{description}.{key} must be a number")
        # rclcpp declares these as double parameters, so force YAML integers to float.
        normalized[key] = float(normalized[key])

    if normalized.get("transport", "usb") not in {"usb", "usb3", "gige", "any"}:
        raise RuntimeError(f"{description}.transport must be usb, usb3, gige, or any")
    if normalized.get("output_encoding", "rgb8") not in {"mono8", "rgb8", "bgr8"}:
        raise RuntimeError(
            f"{description}.output_encoding must be mono8, rgb8, or bgr8"
        )
    if normalized.get("trigger_mode", "off") not in {"off", "software"}:
        raise RuntimeError(f"{description}.trigger_mode must be off or software")
    return normalized


def _valid_namespace(value):
    if not isinstance(value, str):
        return False
    stripped = value.strip("/")
    return not stripped or all(
        _ROS_NAME_TOKEN_PATTERN.fullmatch(token) for token in stripped.split("/")
    )


def _resolve_camera_info_url(value, config_directory):
    if not value or value.startswith(("file://", "package://")):
        return value
    if "://" in value:
        raise RuntimeError(
            "camera_info_url must be a relative path, absolute path, file:// URL, "
            "or package:// URL"
        )

    path = Path(os.path.expandvars(os.path.expanduser(value)))
    if not path.is_absolute():
        path = config_directory / path
    return path.resolve().as_uri()


def _as_launch_parameters(parameters):
    return {
        key: ParameterValue(value, value_type=str)
        if key in _STRING_PARAMETERS
        else value
        for key, value in parameters.items()
    }


def _launch_setup(context):
    cameras_file = LaunchConfiguration("cameras_file").perform(context)
    cameras_path = Path(
        os.path.expandvars(os.path.expanduser(cameras_file))
    ).resolve()
    if not cameras_path.is_file():
        raise RuntimeError(f"Camera configuration file does not exist: {cameras_path}")
    with cameras_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    _require_mapping(document, "camera configuration root")

    defaults = _validate_parameters(
        _require_mapping(document.get("defaults", {}), "defaults"), "defaults"
    )
    cameras = document.get("cameras")
    if not isinstance(cameras, list):
        raise RuntimeError("cameras must be a YAML list")

    nodes = []
    namespaces = set()
    selectors = set()
    for index, raw_camera in enumerate(cameras):
        description = f"cameras[{index}]"
        camera = _require_mapping(raw_camera, description)
        unknown = set(camera) - _DRIVER_PARAMETERS - _LAUNCH_KEYS
        if unknown:
            raise RuntimeError(f"{description} contains unknown keys: {sorted(unknown)}")

        enabled = camera.get("enabled", True)
        if not isinstance(enabled, bool):
            raise RuntimeError(f"{description}.enabled must be true or false")
        if not enabled:
            continue

        namespace = camera.get("namespace", "")
        node_name = camera.get("node_name", "camera")
        if not _valid_namespace(namespace):
            raise RuntimeError(f"{description}.namespace is not a valid ROS namespace")
        if (
            not isinstance(node_name, str)
            or not node_name
            or not _ROS_NAME_TOKEN_PATTERN.fullmatch(node_name)
        ):
            raise RuntimeError(f"{description}.node_name is not a valid ROS node name")

        normalized_namespace = namespace.strip("/")
        if normalized_namespace in namespaces:
            raise RuntimeError(
                f"{description} reuses namespace '{namespace}'; image topics would collide"
            )
        namespaces.add(normalized_namespace)

        explicit_parameters = {
            key: value for key, value in camera.items() if key in _DRIVER_PARAMETERS
        }
        parameters = dict(defaults)
        parameters.update(_validate_parameters(explicit_parameters, description))

        serial = parameters.get("serial_number", "")
        user_name = parameters.get("user_defined_name", "")
        if not serial and not user_name:
            raise RuntimeError(
                f"{description} must set serial_number or user_defined_name"
            )
        selector = ("serial", serial) if serial else ("user_defined_name", user_name)
        if selector in selectors:
            raise RuntimeError(f"{description} reuses camera selector {selector[1]!r}")
        selectors.add(selector)

        default_identity = normalized_namespace.replace("/", "_") or node_name
        parameters.setdefault("transport", "usb")
        parameters.setdefault("camera_name", default_identity)
        parameters.setdefault("camera_info_url", "")
        parameters.setdefault("frame_id", f"{default_identity}_optical_frame")
        parameters.setdefault("output_encoding", "rgb8")
        parameters.setdefault("use_sensor_data_qos", True)
        parameters["camera_info_url"] = _resolve_camera_info_url(
            parameters["camera_info_url"], cameras_path.parent
        )

        nodes.append(
            Node(
                package="hik_camera_driver",
                executable="hik_camera_driver_node",
                namespace=namespace,
                name=node_name,
                output="screen",
                emulate_tty=True,
                parameters=[_as_launch_parameters(parameters)],
            )
        )

    if not nodes:
        return [LogInfo(msg=f"No cameras are enabled in {cameras_file}")]
    return [LogInfo(msg=f"Starting {len(nodes)} cameras from {cameras_file}"), *nodes]


def generate_launch_description():
    package_share = get_package_share_directory("hik_camera_driver")
    default_config = os.path.join(package_share, "config", "cameras.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("cameras_file", default_value=default_config),
            OpaqueFunction(function=_launch_setup),
        ]
    )
