#include "mindvision.hpp"

#include <libusb-1.0/libusb.h>

#include <stdexcept>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
MindVision::MindVision(double exposure_ms, double gamma, double gain, int wb_r, int wb_g, int wb_b, const std::string & vid_pid)
: exposure_ms_(exposure_ms),
  gamma_(gamma),
  gain_(gain),
  wb_r_(wb_r),
  wb_g_(wb_g),
  wb_b_(wb_b),
  handle_(-1),
  quit_(false),
  ok_(false),
  queue_(1),
  vid_(-1),
  pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  try_open();

  // 守护线程
  daemon_thread_ = std::thread{[this] {
    while (!quit_) {
      std::this_thread::sleep_for(100ms);

      if (ok_) continue;

      if (capture_thread_.joinable()) capture_thread_.join();

      close();
      reset_usb();
      try_open();
    }
  }};
}

MindVision::~MindVision()
{
  quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  close();
  tools::logger()->info("Mindvision destructed.");
}

void MindVision::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void MindVision::open()
{
  int camera_num = 1;
  tSdkCameraDevInfo camera_info_list;
  tSdkCameraCapbility camera_capbility;
  CameraSdkInit(1);
  CameraEnumerateDevice(&camera_info_list, &camera_num);

  if (camera_num == 0) throw std::runtime_error("Not found camera!");

  if (CameraInit(&camera_info_list, -1, -1, &handle_) != CAMERA_STATUS_SUCCESS)
    throw std::runtime_error("Failed to init camera!");

  CameraGetCapability(handle_, &camera_capbility);
  width_ = camera_capbility.sResolutionRange.iWidthMax;
  height_ = camera_capbility.sResolutionRange.iHeightMax;

  // 1. 优先加载默认参数（这能把偏色、对比度等问题全部重置为出厂正常状态）
  CameraLoadParameter(handle_, PARAMETER_TEAM_A); 

  // 2. 然后再用我们 yaml 的配置去覆盖特定的设置
  CameraSetAeState(handle_, FALSE);                        // 关闭自动曝光
  CameraSetExposureTime(handle_, exposure_ms_ * 1e3);      // 设置曝光
  CameraSetGamma(handle_, gamma_ * 1e2);                   // 设置伽马
  
  // 3. 彻底关闭自动白平衡，防止 SDK 后台执行算法导致严重的掉帧
  CameraSetWbMode(handle_, FALSE); 
  // 手动设置白平衡（红、绿、蓝 增益）
  CameraSetGain(handle_, wb_r_, wb_g_, wb_b_);

  // 设置模拟增益
  CameraSetAnalogGain(handle_, static_cast<int>(gain_));

  CameraSetIspOutFormat(handle_, CAMERA_MEDIA_TYPE_BGR8);  // 设置输出格式为BGR
  CameraSetTriggerMode(handle_, 0);                        // 设置为连续采集模式
  CameraSetFrameSpeed(handle_, 1);                         // 恢复为原本的 1 (1 可能是高低速的稳定模式，0可能导致USB带宽占满阻塞)

  CameraPlay(handle_);

  // 取图线程
  capture_thread_ = std::thread{[this] {
    tSdkFrameHead head;
    BYTE * raw;

    ok_ = true;
    while (!quit_) {
      std::this_thread::sleep_for(1ms);

      auto img = cv::Mat(height_, width_, CV_8UC3);

      auto status = CameraGetImageBuffer(handle_, &head, &raw, 100);
      auto timestamp = std::chrono::steady_clock::now();

      if (status != CAMERA_STATUS_SUCCESS) {
        tools::logger()->warn("Camera dropped!");
        ok_ = false;
        break;
      }

      CameraImageProcess(handle_, raw, img.data, &head);
      CameraReleaseImageBuffer(handle_, raw);

      queue_.push({img, timestamp});
    }
  }};

  tools::logger()->info("Mindvision opened.");
}

void MindVision::try_open()
{
  try {
    open();
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void MindVision::close()
{
  if (handle_ == -1) return;
  CameraUnInit(handle_);
}

void MindVision::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void MindVision::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
