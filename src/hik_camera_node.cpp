#include "hik_camera_driver/hik_camera_node.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <utility>

namespace hik_camera_driver
{
namespace
{

rcl_interfaces::msg::ParameterDescriptor descriptor(
  const std::string & description, bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = description;
  result.read_only = read_only;
  return result;
}

rcl_interfaces::msg::ParameterDescriptor positiveDoubleDescriptor(const std::string & description)
{
  auto result = descriptor(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = 0.0;
  range.to_value = 1.0e9;
  range.step = 0.0;
  result.floating_point_range.push_back(range);
  return result;
}

rcl_interfaces::msg::ParameterDescriptor integerDescriptor(
  const std::string & description, std::int64_t minimum, std::int64_t maximum,
  bool read_only = true)
{
  auto result = descriptor(description, read_only);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = minimum;
  range.to_value = maximum;
  range.step = 1;
  result.integer_range.push_back(range);
  return result;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string boolString(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
: Node("camera", options), previous_diagnostic_time_(std::chrono::steady_clock::now())
{
  config_ = declareAndReadParameters();

  const auto qos = config_.use_sensor_data_qos ?
    rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  camera_publisher_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  loadCameraInfo();

  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
  diagnostics_timer_ = create_wall_timer(
    std::chrono::seconds(1), std::bind(&HikCameraNode::publishDiagnostics, this));

  trigger_service_ = create_service<std_srvs::srv::Trigger>(
    "~/trigger",
    std::bind(
      &HikCameraNode::triggerCallback, this, std::placeholders::_1, std::placeholders::_2));

  parameters_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

  const auto version = camera_.sdkVersion();
  RCLCPP_INFO(
    get_logger(), "Starting hik_camera_driver; MVS SDK version 0x%08X",
    version);

  capture_thread_ = std::thread(&HikCameraNode::captureLoop, this);
}

HikCameraNode::~HikCameraNode()
{
  stop_requested_.store(true);
  stop_condition_.notify_all();
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  camera_.disconnect();
  RCLCPP_INFO(get_logger(), "hik_camera_driver stopped");
}

HikCameraNode::DriverConfig HikCameraNode::declareAndReadParameters()
{
  DriverConfig config;

  const auto transport = declare_parameter<std::string>(
    "transport", "usb", descriptor("Camera transport: usb, gige, or any", true));
  config.selector.transport = parseTransport(transport);
  config.selector.serial_number = declare_parameter<std::string>(
    "serial_number", "", descriptor("Camera serial number; empty selects the first match", true));
  config.selector.user_defined_name = declare_parameter<std::string>(
    "user_defined_name", "", descriptor("MVS user-defined camera name", true));

  config.camera_name = declare_parameter<std::string>(
    "camera_name", "camera", descriptor("Name stored by camera_info_manager", true));
  config.camera_info_url = declare_parameter<std::string>(
    "camera_info_url", "", descriptor("file:// or package:// camera calibration URL", true));
  config.frame_id = declare_parameter<std::string>(
    "frame_id", "camera_optical_frame", descriptor("Frame ID for image and camera_info", true));
  config.output_encoding = declare_parameter<std::string>(
    "output_encoding", "rgb8", descriptor("Published encoding: mono8, rgb8, or bgr8", true));
  // Validate before starting the worker thread.
  static_cast<void>(encodingSpec(config.output_encoding));

  config.use_sensor_data_qos = declare_parameter<bool>(
    "use_sensor_data_qos", true, descriptor("Use best-effort sensor-data QoS", true));
  config.grab_timeout_ms = declare_parameter<int>(
    "grab_timeout_ms", 1000, integerDescriptor("Frame acquisition timeout", 10, 5000));
  config.reconnect_interval_ms = declare_parameter<int>(
    "reconnect_interval_ms", 1000,
    integerDescriptor("Delay between reconnect attempts", 100, 60000));
  config.max_consecutive_timeouts = declare_parameter<int>(
    "max_consecutive_timeouts", 5,
    integerDescriptor("Timeouts before reconnecting in free-run mode", 1, 1000));
  config.sdk_buffer_count = declare_parameter<int>(
    "sdk_buffer_count", 4, integerDescriptor("MVS internal frame buffers", 1, 30));

  config.camera_settings.auto_exposure = declare_parameter<bool>(
    "auto_exposure", false, descriptor("Enable continuous automatic exposure"));
  config.camera_settings.exposure_time_us = declare_parameter<double>(
    "exposure_time", 6000.0, positiveDoubleDescriptor("Manual exposure time in microseconds"));
  config.camera_settings.auto_gain = declare_parameter<bool>(
    "auto_gain", false, descriptor("Enable continuous automatic gain"));
  config.camera_settings.gain = declare_parameter<double>(
    "gain", 0.0, positiveDoubleDescriptor("Manual camera gain"));
  config.camera_settings.frame_rate = declare_parameter<double>(
    "frame_rate", 0.0,
    positiveDoubleDescriptor("Acquisition frame rate; zero leaves camera default unchanged"));
  const auto trigger_mode = declare_parameter<std::string>(
    "trigger_mode", "off", descriptor("Trigger mode: off or software", true));
  if (trigger_mode != "off" && trigger_mode != "software") {
    throw std::invalid_argument("trigger_mode must be 'off' or 'software'");
  }
  config.camera_settings.software_trigger = trigger_mode == "software";

  return config;
}

void HikCameraNode::loadCameraInfo()
{
  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, config_.camera_name);

  if (config_.camera_info_url.empty()) {
    RCLCPP_WARN(get_logger(), "camera_info_url is empty; publishing uncalibrated CameraInfo");
    return;
  }
  if (!camera_info_manager_->validateURL(config_.camera_info_url)) {
    RCLCPP_ERROR(
      get_logger(), "Invalid camera_info_url: %s", config_.camera_info_url.c_str());
    return;
  }
  if (!camera_info_manager_->loadCameraInfo(config_.camera_info_url)) {
    RCLCPP_ERROR(
      get_logger(), "Failed to load camera calibration: %s", config_.camera_info_url.c_str());
    return;
  }

  calibrated_camera_info_ = camera_info_manager_->getCameraInfo();
  calibration_loaded_ = camera_info_manager_->isCalibrated();
  if (calibration_loaded_) {
    RCLCPP_INFO(
      get_logger(), "Loaded calibration '%s' (%ux%u)", config_.camera_info_url.c_str(),
      calibrated_camera_info_.width, calibrated_camera_info_.height);
  } else {
    RCLCPP_WARN(
      get_logger(), "Calibration file loaded but is not calibrated: %s",
      config_.camera_info_url.c_str());
  }
}

void HikCameraNode::captureLoop()
{
  Frame frame;
  int consecutive_timeouts = 0;

  while (!stop_requested_.load()) {
    if (!camera_.streaming()) {
      if (!openAndStartCamera()) {
        if (waitForStop(std::chrono::milliseconds(config_.reconnect_interval_ms))) {
          break;
        }
        continue;
      }
      consecutive_timeouts = 0;
    }

    std::string error;
    const auto result = camera_.grab(
      frame, config_.output_encoding, static_cast<unsigned int>(config_.grab_timeout_ms), error);
    if (stop_requested_.load()) {
      break;
    }

    if (result == GrabResult::kFrame) {
      consecutive_timeouts = 0;
      lost_packets_.fetch_add(frame.lost_packets);
      publishFrame(frame);
      frames_published_.fetch_add(1);
      last_frame_steady_ns_.store(steadyNowNanoseconds());
      continue;
    }

    if (result == GrabResult::kTimeout) {
      grab_timeouts_.fetch_add(1);
      ++consecutive_timeouts;
      bool software_trigger = false;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        software_trigger = config_.camera_settings.software_trigger;
      }
      if (software_trigger ||
        consecutive_timeouts < config_.max_consecutive_timeouts)
      {
        continue;
      }
      error = "consecutive frame timeouts reached limit: " + error;
    } else {
      grab_errors_.fetch_add(1);
    }

    setState("reconnecting", error);
    RCLCPP_WARN(get_logger(), "%s; reconnecting", error.c_str());
    camera_.disconnect();
    reconnect_attempts_.fetch_add(1);
    calibration_mismatch_.store(false);
    if (waitForStop(std::chrono::milliseconds(config_.reconnect_interval_ms))) {
      break;
    }
  }

  camera_.disconnect();
  setState("stopped");
}

bool HikCameraNode::openAndStartCamera()
{
  setState("connecting");
  std::string error;
  if (!camera_.connect(config_.selector, error)) {
    setState("waiting_for_camera", error);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for camera: %s", error.c_str());
    return false;
  }

  if (camera_.enumeratedDeviceCount() > 1 && config_.selector.serial_number.empty() &&
    config_.selector.user_defined_name.empty())
  {
    RCLCPP_WARN(
      get_logger(), "Found %zu cameras and no selector was configured; using the first device",
      camera_.enumeratedDeviceCount());
  }

  {
    // Keep configuration selection and hardware application atomic with respect to
    // the dynamic-parameter callback.
    std::lock_guard<std::mutex> lock(config_mutex_);
    if (!camera_.applySettings(config_.camera_settings, error)) {
      setState("configuration_error", error);
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      camera_.disconnect();
      return false;
    }
  }
  if (!camera_.start(static_cast<unsigned int>(config_.sdk_buffer_count), error)) {
    setState("start_error", error);
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    camera_.disconnect();
    return false;
  }

  const auto identity = camera_.identity();
  setState("streaming");
  RCLCPP_INFO(
    get_logger(), "Streaming %s camera model='%s' serial='%s' user_name='%s'%s%s",
    transportName(identity.transport).c_str(), identity.model_name.c_str(),
    identity.serial_number.c_str(), identity.user_defined_name.c_str(),
    identity.ip_address.empty() ? "" : " ip='",
    identity.ip_address.empty() ? "" : (identity.ip_address + "'").c_str());
  return true;
}

void HikCameraNode::publishFrame(Frame & frame)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = now();
  image.header.frame_id = config_.frame_id;
  image.height = frame.height;
  image.width = frame.width;
  image.encoding = frame.encoding;
  image.is_bigendian = false;
  image.step = frame.step;
  image.data = std::move(frame.data);

  auto camera_info = cameraInfoFor(frame.width, frame.height, image.header);
  camera_publisher_.publish(image, camera_info);

  // Reuse the allocated image buffer on the next acquisition.
  frame.data = std::move(image.data);
}

sensor_msgs::msg::CameraInfo HikCameraNode::cameraInfoFor(
  std::uint32_t width, std::uint32_t height, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::CameraInfo info;
  if (calibration_loaded_ && calibrated_camera_info_.width == width &&
    calibrated_camera_info_.height == height)
  {
    info = calibrated_camera_info_;
  } else {
    info.width = width;
    info.height = height;
    if (calibration_loaded_ && !calibration_mismatch_.exchange(true)) {
      RCLCPP_ERROR(
        get_logger(),
        "Calibration resolution is %ux%u but frames are %ux%u; publishing uncalibrated CameraInfo",
        calibrated_camera_info_.width, calibrated_camera_info_.height, width, height);
    }
  }
  info.header = header;
  return info;
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::unique_lock<std::mutex> config_lock(config_mutex_);
  const CameraSettings previous = config_.camera_settings;
  CameraSettings next = previous;

  bool camera_setting_changed = false;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "auto_exposure") {
      next.auto_exposure = parameter.as_bool();
      camera_setting_changed = true;
    } else if (name == "exposure_time") {
      next.exposure_time_us = parameter.as_double();
      camera_setting_changed = true;
    } else if (name == "auto_gain") {
      next.auto_gain = parameter.as_bool();
      camera_setting_changed = true;
    } else if (name == "gain") {
      next.gain = parameter.as_double();
      camera_setting_changed = true;
    } else if (name == "frame_rate") {
      next.frame_rate = parameter.as_double();
      camera_setting_changed = true;
    }
  }

  if (!camera_setting_changed) {
    return result;
  }
  if (next.exposure_time_us <= 0.0) {
    result.successful = false;
    result.reason = "exposure_time must be greater than zero";
    return result;
  }
  if (next.gain < 0.0 || next.frame_rate < 0.0) {
    result.successful = false;
    result.reason = "gain and frame_rate must not be negative";
    return result;
  }

  if (camera_.connected()) {
    std::string error;
    if (!camera_.applySettings(next, error)) {
      std::string rollback_error;
      camera_.applySettings(previous, rollback_error);
      result.successful = false;
      result.reason = error;
      if (!rollback_error.empty()) {
        result.reason += "; rollback also failed: " + rollback_error;
      }
      return result;
    }
  }

  config_.camera_settings = next;
  return result;
}

void HikCameraNode::triggerCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  bool software_trigger = false;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    software_trigger = config_.camera_settings.software_trigger;
  }
  if (!software_trigger) {
    response->success = false;
    response->message = "trigger_mode is not 'software'";
    return;
  }
  response->success = camera_.softwareTrigger(response->message);
  if (response->success) {
    response->message = "software trigger sent";
  }
}

void HikCameraNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string(": camera");

  std::string state;
  std::string last_error;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = state_;
    last_error = last_error_;
  }
  if (state == "streaming") {
    status.level = calibration_mismatch_.load() ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = calibration_mismatch_.load() ?
      "streaming; calibration resolution mismatch" : "streaming";
  } else if (state == "starting" || state == "connecting" ||
    state == "waiting_for_camera" || state == "reconnecting")
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = last_error.empty() ? state : state + ": " + last_error;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = last_error.empty() ? state : state + ": " + last_error;
  }

  const auto identity = camera_.identity();
  if (!identity.serial_number.empty()) {
    status.hardware_id = identity.serial_number;
  } else if (!config_.selector.serial_number.empty()) {
    status.hardware_id = config_.selector.serial_number;
  } else {
    status.hardware_id = config_.selector.user_defined_name;
  }
  const bool have_identity = !identity.serial_number.empty() || !identity.model_name.empty();
  const auto diagnostic_transport =
    have_identity ? identity.transport : config_.selector.transport;

  const auto now_steady = std::chrono::steady_clock::now();
  const auto frame_count = frames_published_.load();
  const double interval =
    std::chrono::duration<double>(now_steady - previous_diagnostic_time_).count();
  const double fps = interval > 0.0 ?
    static_cast<double>(frame_count - previous_diagnostic_frame_count_) / interval : 0.0;
  previous_diagnostic_time_ = now_steady;
  previous_diagnostic_frame_count_ = frame_count;

  const auto last_frame_ns = last_frame_steady_ns_.load();
  const double frame_age = last_frame_ns == 0 ? -1.0 :
    static_cast<double>(steadyNowNanoseconds() - last_frame_ns) / 1.0e9;

  status.values.push_back(keyValue("state", state));
  status.values.push_back(keyValue("transport", transportName(diagnostic_transport)));
  status.values.push_back(keyValue("model", identity.model_name));
  status.values.push_back(keyValue("serial_number", identity.serial_number));
  status.values.push_back(keyValue("user_defined_name", identity.user_defined_name));
  status.values.push_back(keyValue("ip_address", identity.ip_address));
  status.values.push_back(keyValue("frames_published", std::to_string(frame_count)));
  status.values.push_back(keyValue("current_fps", std::to_string(fps)));
  status.values.push_back(keyValue("last_frame_age_sec", std::to_string(frame_age)));
  status.values.push_back(keyValue("grab_timeouts", std::to_string(grab_timeouts_.load())));
  status.values.push_back(keyValue("grab_errors", std::to_string(grab_errors_.load())));
  status.values.push_back(keyValue("lost_packets", std::to_string(lost_packets_.load())));
  status.values.push_back(
    keyValue("reconnect_attempts", std::to_string(reconnect_attempts_.load())));
  status.values.push_back(keyValue("calibration_loaded", boolString(calibration_loaded_)));
  status.values.push_back(
    keyValue("calibration_mismatch", boolString(calibration_mismatch_.load())));
  status.values.push_back(keyValue("output_encoding", config_.output_encoding));

  message.status.push_back(std::move(status));
  diagnostics_publisher_->publish(message);
}

void HikCameraNode::setState(const std::string & state, const std::string & error)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = state;
  last_error_ = error;
}

bool HikCameraNode::waitForStop(std::chrono::milliseconds duration)
{
  std::unique_lock<std::mutex> lock(stop_mutex_);
  return stop_condition_.wait_for(lock, duration, [this]() { return stop_requested_.load(); });
}

std::int64_t HikCameraNode::steadyNowNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace hik_camera_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera_driver::HikCameraNode)
