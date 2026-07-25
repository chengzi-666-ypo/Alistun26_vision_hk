#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  
  std::string vid_pid;
  try {
    vid_pid = tools::read<std::string>(yaml, "vid_pid");
  } catch (...) {
    vid_pid = "";
  }
  
  std::string camera_name = "auto";
  try {
    camera_name = tools::read<std::string>(yaml, "camera_name");
  } catch (...) {}

  int mv_camera_num = 0;
  int hik_camera_num = 0;

  if (camera_name == "auto" || camera_name == "mindvision") {
    // Check MindVision cameras
    mv_camera_num = 1;
    tSdkCameraDevInfo mv_camera_info_list;
    CameraSdkInit(1);
    CameraEnumerateDevice(&mv_camera_info_list, &mv_camera_num);
  }

  if (camera_name == "auto" || camera_name == "hikrobot") {
    // Check HikRobot cameras
    MV_CC_DEVICE_INFO_LIST hik_device_list;
    unsigned int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &hik_device_list);
    hik_camera_num = (ret == MV_OK) ? hik_device_list.nDeviceNum : 0;
  }

  if (camera_name == "auto" && mv_camera_num > 0 && hik_camera_num > 0) {
    throw std::runtime_error("Both MindVision and HikRobot cameras detected! Only one can be connected.");
  } else if (mv_camera_num > 0 || camera_name == "mindvision") {
    // std::cout or logger could be used here, assume logger is available via tools
    double mv_exposure_ms = exposure_ms;
    double mv_gamma = 1.0;
    double mv_gain = 2.0;
    int mv_wb_r = 100;
    int mv_wb_g = 100;
    int mv_wb_b = 100;
    try {
      mv_exposure_ms = tools::read<double>(yaml, "mv_exposure_ms");
    } catch (...) {}
    try {
      mv_gamma = tools::read<double>(yaml, "mv_gamma");
    } catch (...) {}
    try {
      mv_gain = tools::read<double>(yaml, "mv_gain");
    } catch (...) {}
    try {
      mv_wb_r = tools::read<int>(yaml, "mv_wb_r");
      mv_wb_g = tools::read<int>(yaml, "mv_wb_g");
      mv_wb_b = tools::read<int>(yaml, "mv_wb_b");
    } catch (...) {}
    camera_ = std::make_unique<MindVision>(mv_exposure_ms, mv_gamma, mv_gain, mv_wb_r, mv_wb_g, mv_wb_b, vid_pid);
  } else if (hik_camera_num > 0 || camera_name == "hikrobot") {
    double gain = 10.0;
    try {
      gain = tools::read<double>(yaml, "gain");
    } catch (...) {}
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  } else {
    throw std::runtime_error("No camera detected!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
