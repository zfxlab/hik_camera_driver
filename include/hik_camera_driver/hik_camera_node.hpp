#pragma once

#include "hik_camera_driver/camera_device.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <image_transport/camera_publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hik_camera_driver
{

class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options);
  ~HikCameraNode() override;

private:
  struct DriverConfig
  {
    DeviceSelector selector;
    CameraSettings camera_settings;
    std::string camera_name;
    std::string camera_info_url;
    std::string frame_id;
    std::string output_encoding;
    bool use_sensor_data_qos{true};
    int grab_timeout_ms{1000};
    int reconnect_interval_ms{1000};
    int max_consecutive_timeouts{5};
    int sdk_buffer_count{4};
  };

  DriverConfig declareAndReadParameters();
  void loadCameraInfo();
  void captureLoop();
  bool openAndStartCamera();
  void publishFrame(Frame & frame);
  sensor_msgs::msg::CameraInfo cameraInfoFor(
    std::uint32_t width, std::uint32_t height, const std_msgs::msg::Header & header);

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  void triggerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void publishDiagnostics();

  void setState(const std::string & state, const std::string & error = "");
  bool waitForStop(std::chrono::milliseconds duration);
  static std::int64_t steadyNowNanoseconds();

  CameraDevice camera_;
  DriverConfig config_;
  std::mutex config_mutex_;

  image_transport::CameraPublisher camera_publisher_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo calibrated_camera_info_;
  bool calibration_loaded_{false};
  std::atomic<bool> calibration_mismatch_{false};

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

  std::atomic<bool> stop_requested_{false};
  std::thread capture_thread_;
  std::mutex stop_mutex_;
  std::condition_variable stop_condition_;

  mutable std::mutex state_mutex_;
  std::string state_{"starting"};
  std::string last_error_;

  std::atomic<std::uint64_t> frames_published_{0};
  std::atomic<std::uint64_t> grab_timeouts_{0};
  std::atomic<std::uint64_t> grab_errors_{0};
  std::atomic<std::uint64_t> reconnect_attempts_{0};
  std::atomic<std::uint64_t> lost_packets_{0};
  std::atomic<std::int64_t> last_frame_steady_ns_{0};
  std::uint64_t previous_diagnostic_frame_count_{0};
  std::chrono::steady_clock::time_point previous_diagnostic_time_;
};

}  // namespace hik_camera_driver
