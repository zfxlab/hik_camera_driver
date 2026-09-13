#include "hik_camera_driver/camera_device.hpp"

#include <MvErrorDefine.h>

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace hik_camera_driver {
namespace {

class RealSdkApi final : public SdkApi {
 public:
  unsigned int sdkVersion() override { return MV_CC_GetSDKVersion(); }
  int enumDevices(unsigned int mask, MV_CC_DEVICE_INFO_LIST * list) override {
    return MV_CC_EnumDevices(mask, list);
  }
  int createHandle(void ** handle, const MV_CC_DEVICE_INFO * info) override {
    return MV_CC_CreateHandle(handle, info);
  }
  int destroyHandle(void ** handle) override { return MV_CC_DestroyHandle(handle); }
  int openDevice(void * handle) override { return MV_CC_OpenDevice(handle); }
  int closeDevice(void * handle) override { return MV_CC_CloseDevice(handle); }
  bool isDeviceConnected(void * handle) override { return MV_CC_IsDeviceConnected(handle); }
  int startGrabbing(void * handle) override { return MV_CC_StartGrabbing(handle); }
  int stopGrabbing(void * handle) override { return MV_CC_StopGrabbing(handle); }
  int setImageNodeNum(void * handle, unsigned int count) override {
    return MV_CC_SetImageNodeNum(handle, count);
  }
  int getImageBuffer(void * handle, MV_FRAME_OUT * frame, unsigned int timeout_ms) override {
    return MV_CC_GetImageBuffer(handle, frame, timeout_ms);
  }
  int freeImageBuffer(void * handle, MV_FRAME_OUT * frame) override {
    return MV_CC_FreeImageBuffer(handle, frame);
  }
  int convertPixelType(void * handle, MV_CC_PIXEL_CONVERT_PARAM * params) override {
    return MV_CC_ConvertPixelType(handle, params);
  }
  int setFloatValue(void * handle, const char * key, float value) override {
    return MV_CC_SetFloatValue(handle, key, value);
  }
  int setBoolValue(void * handle, const char * key, bool value) override {
    return MV_CC_SetBoolValue(handle, key, value);
  }
  int setEnumValueByString(void * handle, const char * key, const char * value) override {
    return MV_CC_SetEnumValueByString(handle, key, value);
  }
  int setIntValue(void * handle, const char * key, unsigned int value) override {
    return MV_CC_SetIntValue(handle, key, value);
  }
  int setCommandValue(void * handle, const char * key) override {
    return MV_CC_SetCommandValue(handle, key);
  }
  int getOptimalPacketSize(void * handle) override { return MV_CC_GetOptimalPacketSize(handle); }
};

template <std::size_t N>
std::string boundedString(const unsigned char (&value)[N]) {
  const auto * begin = reinterpret_cast<const char *>(value);
  const auto * end = std::find(begin, begin + N, '\0');
  return std::string(begin, end);
}

std::string lowerCopy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

unsigned int transportMask(Transport transport) {
  switch (transport) {
    case Transport::kUsb:
      return MV_USB_DEVICE;
    case Transport::kGigE:
      return MV_GIGE_DEVICE;
    case Transport::kAny:
      return MV_USB_DEVICE | MV_GIGE_DEVICE;
  }
  return MV_USB_DEVICE;
}

DeviceIdentity identityFrom(const MV_CC_DEVICE_INFO & info) {
  DeviceIdentity identity;
  if (info.nTLayerType == MV_GIGE_DEVICE) {
    identity.transport = Transport::kGigE;
    identity.serial_number = boundedString(info.SpecialInfo.stGigEInfo.chSerialNumber);
    identity.user_defined_name = boundedString(info.SpecialInfo.stGigEInfo.chUserDefinedName);
    identity.model_name = boundedString(info.SpecialInfo.stGigEInfo.chModelName);
    const auto ip = info.SpecialInfo.stGigEInfo.nCurrentIp;
    std::ostringstream stream;
    stream << ((ip >> 24U) & 0xffU) << '.' << ((ip >> 16U) & 0xffU) << '.' << ((ip >> 8U) & 0xffU)
           << '.' << (ip & 0xffU);
    identity.ip_address = stream.str();
  } else {
    identity.transport = Transport::kUsb;
    identity.serial_number = boundedString(info.SpecialInfo.stUsb3VInfo.chSerialNumber);
    identity.user_defined_name = boundedString(info.SpecialInfo.stUsb3VInfo.chUserDefinedName);
    identity.model_name = boundedString(info.SpecialInfo.stUsb3VInfo.chModelName);
  }
  return identity;
}

bool matches(const DeviceIdentity & identity, const DeviceSelector & selector) {
  if (!selector.serial_number.empty() && identity.serial_number != selector.serial_number) {
    return false;
  }
  if (!selector.user_defined_name.empty() &&
      identity.user_defined_name != selector.user_defined_name) {
    return false;
  }
  return true;
}

bool isTimeoutCode(int code) {
  const auto value = static_cast<unsigned int>(code);
  return value == MV_E_NODATA || value == MV_E_GC_TIMEOUT;
}

class FrameBufferGuard {
 public:
  FrameBufferGuard(SdkApi & api, void * handle, MV_FRAME_OUT & frame)
      : api_(api), handle_(handle), frame_(frame) {}

  ~FrameBufferGuard() { api_.freeImageBuffer(handle_, &frame_); }

  FrameBufferGuard(const FrameBufferGuard &) = delete;
  FrameBufferGuard & operator=(const FrameBufferGuard &) = delete;

 private:
  SdkApi & api_;
  void * handle_;
  MV_FRAME_OUT & frame_;
};

}  // namespace

std::shared_ptr<SdkApi> makeRealSdkApi() { return std::make_shared<RealSdkApi>(); }

Transport parseTransport(const std::string & value) {
  const auto normalized = lowerCopy(value);
  if (normalized == "usb" || normalized == "usb3") {
    return Transport::kUsb;
  }
  if (normalized == "gige" || normalized == "gigabit_ethernet") {
    return Transport::kGigE;
  }
  if (normalized == "any") {
    return Transport::kAny;
  }
  throw std::invalid_argument("transport must be one of: usb, gige, any");
}

std::string transportName(Transport transport) {
  switch (transport) {
    case Transport::kUsb:
      return "usb";
    case Transport::kGigE:
      return "gige";
    case Transport::kAny:
      return "any";
  }
  return "unknown";
}

EncodingSpec encodingSpec(const std::string & ros_encoding) {
  const auto normalized = lowerCopy(ros_encoding);
  if (normalized == "mono8") {
    return {"mono8", PixelType_Gvsp_Mono8, 1};
  }
  if (normalized == "rgb8") {
    return {"rgb8", PixelType_Gvsp_RGB8_Packed, 3};
  }
  if (normalized == "bgr8") {
    return {"bgr8", PixelType_Gvsp_BGR8_Packed, 3};
  }
  throw std::invalid_argument("output_encoding must be one of: mono8, rgb8, bgr8");
}

std::string sdkErrorToString(int code) {
  if (code == MV_OK) {
    return "success";
  }

  const auto value = static_cast<unsigned int>(code);
  const char * description = "unknown SDK error";
  switch (value) {
    case MV_E_HANDLE:
      description = "invalid handle";
      break;
    case MV_E_SUPPORT:
      description = "unsupported function";
      break;
    case MV_E_BUFOVER:
      description = "buffer overflow";
      break;
    case MV_E_CALLORDER:
      description = "invalid call order";
      break;
    case MV_E_PARAMETER:
      description = "invalid parameter";
      break;
    case MV_E_RESOURCE:
      description = "resource allocation failed";
      break;
    case MV_E_NODATA:
      description = "no frame data";
      break;
    case MV_E_NOENOUGH_BUF:
      description = "insufficient buffer";
      break;
    case MV_E_GC_TIMEOUT:
      description = "GenICam timeout";
      break;
    case MV_E_ACCESS_DENIED:
      description = "device access denied; check USB permissions or another exclusive client";
      break;
    case MV_E_BUSY:
      description = "device busy or disconnected";
      break;
    case MV_E_USB_DEVICE:
      description = "USB device error";
      break;
    case MV_E_USB_BANDWIDTH:
      description = "insufficient USB bandwidth";
      break;
    case MV_E_USB_DRIVER:
      description = "USB driver mismatch or unavailable";
      break;
    default:
      break;
  }

  std::ostringstream stream;
  stream << description << " (0x" << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
         << value << ')';
  return stream.str();
}

CameraDevice::CameraDevice(std::shared_ptr<SdkApi> api) : api_(std::move(api)) {
  if (!api_) {
    throw std::invalid_argument("SdkApi must not be null");
  }
}

CameraDevice::~CameraDevice() { disconnect(); }

bool CameraDevice::connect(const DeviceSelector & selector, std::string & error) {
  std::lock_guard<std::mutex> lock(mutex_);
  disconnectUnlocked();

  MV_CC_DEVICE_INFO_LIST list{};
  const int enum_status = api_->enumDevices(transportMask(selector.transport), &list);
  if (enum_status != MV_OK) {
    error = "device enumeration failed: " + sdkErrorToString(enum_status);
    return false;
  }
  enumerated_device_count_ = list.nDeviceNum;
  if (list.nDeviceNum == 0) {
    error = "no matching transport devices found";
    return false;
  }

  const MV_CC_DEVICE_INFO * selected = nullptr;
  DeviceIdentity selected_identity;
  for (unsigned int index = 0; index < list.nDeviceNum; ++index) {
    if (list.pDeviceInfo[index] == nullptr) {
      continue;
    }
    const auto candidate = identityFrom(*list.pDeviceInfo[index]);
    if (matches(candidate, selector)) {
      selected = list.pDeviceInfo[index];
      selected_identity = candidate;
      break;
    }
  }

  if (selected == nullptr) {
    error = "no camera matched serial_number='" + selector.serial_number +
            "' and user_defined_name='" + selector.user_defined_name + "'";
    return false;
  }

  void * new_handle = nullptr;
  int status = api_->createHandle(&new_handle, selected);
  if (status != MV_OK) {
    if (new_handle != nullptr) {
      api_->destroyHandle(&new_handle);
    }
    error = "create camera handle failed: " + sdkErrorToString(status);
    return false;
  }
  handle_ = new_handle;

  status = api_->openDevice(handle_);
  if (status != MV_OK) {
    error = "open camera failed: " + sdkErrorToString(status);
    api_->destroyHandle(&handle_);
    handle_ = nullptr;
    return false;
  }

  identity_ = selected_identity;
  if (identity_.transport == Transport::kGigE) {
    const int packet_size = api_->getOptimalPacketSize(handle_);
    if (packet_size > 0) {
      // Packet-size tuning is an optimization. An unsupported node must not make opening fail.
      api_->setIntValue(handle_, "GevSCPSPacketSize", static_cast<unsigned int>(packet_size));
    }
  }

  error.clear();
  return true;
}

void CameraDevice::disconnect() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  disconnectUnlocked();
}

void CameraDevice::disconnectUnlocked() noexcept {
  if (handle_ == nullptr) {
    streaming_ = false;
    return;
  }
  if (streaming_) {
    api_->stopGrabbing(handle_);
    streaming_ = false;
  }
  api_->closeDevice(handle_);
  api_->destroyHandle(&handle_);
  handle_ = nullptr;
}

bool CameraDevice::setEnumUnlocked(const char * key, const char * value, std::string & error) {
  const int status = api_->setEnumValueByString(handle_, key, value);
  if (status != MV_OK) {
    error = std::string("set ") + key + "=" + value + " failed: " + sdkErrorToString(status);
    return false;
  }
  return true;
}

bool CameraDevice::setFloatUnlocked(const char * key, double value, std::string & error) {
  const int status = api_->setFloatValue(handle_, key, static_cast<float>(value));
  if (status != MV_OK) {
    error = std::string("set ") + key + " failed: " + sdkErrorToString(status);
    return false;
  }
  return true;
}

bool CameraDevice::applySettings(const CameraSettings & settings, std::string & error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    error = "camera is not connected";
    return false;
  }

  if (!setEnumUnlocked("ExposureAuto", settings.auto_exposure ? "Continuous" : "Off", error)) {
    return false;
  }
  if (!settings.auto_exposure &&
      !setFloatUnlocked("ExposureTime", settings.exposure_time_us, error)) {
    return false;
  }
  if (!setEnumUnlocked("GainAuto", settings.auto_gain ? "Continuous" : "Off", error)) {
    return false;
  }
  if (!settings.auto_gain && !setFloatUnlocked("Gain", settings.gain, error)) {
    return false;
  }
  if (settings.frame_rate > 0.0) {
    const int enable_status = api_->setBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    if (enable_status != MV_OK) {
      error = "enable acquisition frame rate failed: " + sdkErrorToString(enable_status);
      return false;
    }
    if (!setFloatUnlocked("AcquisitionFrameRate", settings.frame_rate, error)) {
      return false;
    }
  }

  if (!setEnumUnlocked("TriggerMode", settings.software_trigger ? "On" : "Off", error)) {
    return false;
  }
  if (settings.software_trigger && !setEnumUnlocked("TriggerSource", "Software", error)) {
    return false;
  }

  error.clear();
  return true;
}

bool CameraDevice::start(unsigned int sdk_buffer_count, std::string & error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    error = "camera is not connected";
    return false;
  }
  if (streaming_) {
    error.clear();
    return true;
  }
  const int buffer_status = api_->setImageNodeNum(handle_, sdk_buffer_count);
  if (buffer_status != MV_OK) {
    error = "set SDK buffer count failed: " + sdkErrorToString(buffer_status);
    return false;
  }
  const int status = api_->startGrabbing(handle_);
  if (status != MV_OK) {
    error = "start acquisition failed: " + sdkErrorToString(status);
    return false;
  }
  streaming_ = true;
  error.clear();
  return true;
}

void CameraDevice::stop() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr && streaming_) {
    api_->stopGrabbing(handle_);
  }
  streaming_ = false;
}

bool CameraDevice::softwareTrigger(std::string & error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr || !streaming_) {
    error = "camera is not streaming";
    return false;
  }
  const int status = api_->setCommandValue(handle_, "TriggerSoftware");
  if (status != MV_OK) {
    error = "software trigger failed: " + sdkErrorToString(status);
    return false;
  }
  error.clear();
  return true;
}

GrabResult CameraDevice::grab(Frame & frame, const std::string & output_encoding,
                              unsigned int timeout_ms, std::string & error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr || !streaming_) {
    error = "camera is not streaming";
    return GrabResult::kDisconnected;
  }

  MV_FRAME_OUT raw_frame{};
  const int grab_status = api_->getImageBuffer(handle_, &raw_frame, timeout_ms);
  if (grab_status != MV_OK) {
    if (isTimeoutCode(grab_status)) {
      error = sdkErrorToString(grab_status);
      return GrabResult::kTimeout;
    }
    if (!api_->isDeviceConnected(handle_)) {
      error = "camera disconnected: " + sdkErrorToString(grab_status);
      return GrabResult::kDisconnected;
    }
    error = "get image buffer failed: " + sdkErrorToString(grab_status);
    return GrabResult::kError;
  }
  frame.received_at = std::chrono::steady_clock::now();
  FrameBufferGuard frame_guard(*api_, handle_, raw_frame);

  EncodingSpec spec;
  try {
    spec = encodingSpec(output_encoding);
  } catch (const std::exception & exception) {
    error = exception.what();
    return GrabResult::kError;
  }

  const std::size_t width = raw_frame.stFrameInfo.nWidth;
  const std::size_t height = raw_frame.stFrameInfo.nHeight;
  if (width == 0 || height == 0 || raw_frame.pBufAddr == nullptr) {
    error = "SDK returned an empty frame";
    return GrabResult::kError;
  }
  if (width > std::numeric_limits<std::size_t>::max() / height ||
      width * height > std::numeric_limits<std::size_t>::max() / spec.bytes_per_pixel) {
    error = "frame dimensions overflow host size_t";
    return GrabResult::kError;
  }

  const std::size_t output_size = width * height * spec.bytes_per_pixel;
  if (output_size > std::numeric_limits<unsigned int>::max()) {
    error = "frame is larger than the MVS conversion API supports";
    return GrabResult::kError;
  }
  frame.data.resize(output_size);

  const bool direct_copy = raw_frame.stFrameInfo.enPixelType == spec.mvs_pixel_type &&
                           raw_frame.stFrameInfo.nFrameLen == output_size;
  if (direct_copy) {
    std::copy_n(raw_frame.pBufAddr, output_size, frame.data.begin());
  } else {
    MV_CC_PIXEL_CONVERT_PARAM params{};
    params.nWidth = raw_frame.stFrameInfo.nWidth;
    params.nHeight = raw_frame.stFrameInfo.nHeight;
    params.enSrcPixelType = raw_frame.stFrameInfo.enPixelType;
    params.pSrcData = raw_frame.pBufAddr;
    params.nSrcDataLen = raw_frame.stFrameInfo.nFrameLen;
    params.enDstPixelType = spec.mvs_pixel_type;
    params.pDstBuffer = frame.data.data();
    params.nDstBufferSize = static_cast<unsigned int>(frame.data.size());

    const int convert_status = api_->convertPixelType(handle_, &params);
    if (convert_status != MV_OK) {
      error = "pixel conversion failed: " + sdkErrorToString(convert_status);
      return GrabResult::kError;
    }
    if (params.nDstLen != output_size) {
      error = "pixel conversion returned " + std::to_string(params.nDstLen) + " bytes; expected " +
              std::to_string(output_size);
      return GrabResult::kError;
    }
  }

  frame.width = static_cast<std::uint32_t>(width);
  frame.height = static_cast<std::uint32_t>(height);
  frame.step = frame.width * spec.bytes_per_pixel;
  frame.frame_number = raw_frame.stFrameInfo.nFrameNum;
  frame.lost_packets = raw_frame.stFrameInfo.nLostPacket;
  frame.encoding = spec.ros_encoding;
  error.clear();
  return GrabResult::kFrame;
}

bool CameraDevice::connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return handle_ != nullptr && api_->isDeviceConnected(handle_);
}

bool CameraDevice::streaming() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return handle_ != nullptr && streaming_;
}

DeviceIdentity CameraDevice::identity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return identity_;
}

std::size_t CameraDevice::enumeratedDeviceCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return enumerated_device_count_;
}

unsigned int CameraDevice::sdkVersion() const { return api_->sdkVersion(); }

}  // namespace hik_camera_driver
