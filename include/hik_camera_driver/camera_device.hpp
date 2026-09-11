#include <chrono>
#pragma once

#include <MvCameraControl.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace hik_camera_driver
{

enum class Transport
{
  kUsb,
  kGigE,
  kAny,
};

struct DeviceSelector
{
  Transport transport{Transport::kUsb};
  std::string serial_number;
  std::string user_defined_name;
};

struct DeviceIdentity
{
  Transport transport{Transport::kUsb};
  std::string serial_number;
  std::string user_defined_name;
  std::string model_name;
  std::string ip_address;
};

struct CameraSettings
{
  bool auto_exposure{false};
  double exposure_time_us{6000.0};
  bool auto_gain{false};
  double gain{0.0};
  double frame_rate{0.0};
  bool software_trigger{false};
};

struct EncodingSpec
{
  std::string ros_encoding;
  MvGvspPixelType mvs_pixel_type{PixelType_Gvsp_Undefined};
  std::uint32_t bytes_per_pixel{0};
};

struct Frame
{
  std::vector<std::uint8_t> data;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t step{0};
  std::uint32_t frame_number{0};
  std::chrono::steady_clock::time_point received_at{};
  std::uint32_t lost_packets{0};
  std::string encoding;
};

enum class GrabResult
{
  kFrame,
  kTimeout,
  kDisconnected,
  kError,
};

class SdkApi
{
public:
  virtual ~SdkApi() = default;

  virtual unsigned int sdkVersion() = 0;
  virtual int enumDevices(unsigned int transport_mask, MV_CC_DEVICE_INFO_LIST * list) = 0;
  virtual int createHandle(void ** handle, const MV_CC_DEVICE_INFO * info) = 0;
  virtual int destroyHandle(void ** handle) = 0;
  virtual int openDevice(void * handle) = 0;
  virtual int closeDevice(void * handle) = 0;
  virtual bool isDeviceConnected(void * handle) = 0;
  virtual int startGrabbing(void * handle) = 0;
  virtual int stopGrabbing(void * handle) = 0;
  virtual int setImageNodeNum(void * handle, unsigned int count) = 0;
  virtual int getImageBuffer(void * handle, MV_FRAME_OUT * frame, unsigned int timeout_ms) = 0;
  virtual int freeImageBuffer(void * handle, MV_FRAME_OUT * frame) = 0;
  virtual int convertPixelType(void * handle, MV_CC_PIXEL_CONVERT_PARAM * params) = 0;
  virtual int setFloatValue(void * handle, const char * key, float value) = 0;
  virtual int setBoolValue(void * handle, const char * key, bool value) = 0;
  virtual int setEnumValueByString(void * handle, const char * key, const char * value) = 0;
  virtual int setIntValue(void * handle, const char * key, unsigned int value) = 0;
  virtual int setCommandValue(void * handle, const char * key) = 0;
  virtual int getOptimalPacketSize(void * handle) = 0;
};

std::shared_ptr<SdkApi> makeRealSdkApi();

Transport parseTransport(const std::string & value);
std::string transportName(Transport transport);
EncodingSpec encodingSpec(const std::string & ros_encoding);
std::string sdkErrorToString(int code);

class CameraDevice
{
public:
  explicit CameraDevice(std::shared_ptr<SdkApi> api = makeRealSdkApi());
  ~CameraDevice();

  CameraDevice(const CameraDevice &) = delete;
  CameraDevice & operator=(const CameraDevice &) = delete;

  bool connect(const DeviceSelector & selector, std::string & error);
  void disconnect() noexcept;

  bool applySettings(const CameraSettings & settings, std::string & error);
  bool start(unsigned int sdk_buffer_count, std::string & error);
  void stop() noexcept;
  bool softwareTrigger(std::string & error);

  GrabResult grab(
    Frame & frame, const std::string & output_encoding, unsigned int timeout_ms,
    std::string & error);

  bool connected() const;
  bool streaming() const;
  DeviceIdentity identity() const;
  std::size_t enumeratedDeviceCount() const;
  unsigned int sdkVersion() const;

private:
  void disconnectUnlocked() noexcept;
  bool setEnumUnlocked(const char * key, const char * value, std::string & error);
  bool setFloatUnlocked(const char * key, double value, std::string & error);

  std::shared_ptr<SdkApi> api_;
  mutable std::mutex mutex_;
  void * handle_{nullptr};
  bool streaming_{false};
  DeviceIdentity identity_;
  std::size_t enumerated_device_count_{0};
};

}  // namespace hik_camera_driver
