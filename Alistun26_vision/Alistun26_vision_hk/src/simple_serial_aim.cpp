#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

// 串口相关
#include "serial/serial.h"
#include "io/cboard_comm.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/standard_serial.yaml | yaml配置文件的路径}"
  "{tradition t    | false             | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("config-path");
  auto use_tradition = cli.get<bool>("tradition");

  tools::logger()->info("Attempting to load config from: {}", config_path);
  char cwd[1024];
  if (getcwd(cwd, sizeof(cwd)) != NULL) {
      tools::logger()->info("Current working directory: {}", cwd);
  }

  tools::Plotter plotter;
  tools::Exiter exiter;

  // 1. 初始化串口
  std::unique_ptr<serial::Serial> serial_ptr;
  uint8_t send_id = 0xFF;
  try {
    auto yaml = tools::load(config_path);
    std::string serial_port = "/dev/cboard";
    int serial_baud = 115200;
    
    if (yaml["serial_port"]) serial_port = yaml["serial_port"].as<std::string>();
    if (yaml["serial_baud"]) serial_baud = yaml["serial_baud"].as<int>();
    if (yaml["send_canid"]) send_id = static_cast<uint8_t>(yaml["send_canid"].as<int>() & 0xFF);

    serial_ptr.reset(new serial::Serial(serial_port, static_cast<uint32_t>(serial_baud), serial::Timeout::simpleTimeout(10)));
    if (serial_ptr->isOpen()) {
      tools::logger()->info("Serial opened: {} @ {}", serial_port, serial_baud);
    }
  } catch (const std::exception & e) {
    tools::logger()->error("Failed to open serial: {}", e.what());
    return -1;
  }

  // 2. 初始化相机和识别组件
  io::Camera camera(config_path);
  auto_aim::YOLO yolo(config_path);
  auto_aim::Detector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();

  io::Command last_command;
  int frame_count = 0;

  tools::logger()->info("Starting main loop...");

  while (!exiter.exit()) {
    std::chrono::steady_clock::time_point timestamp;
    camera.read(img, timestamp);
    frame_count++;

    if (img.empty()) {
      tools::logger()->warn("Empty image received");
      continue;
    }

    // 默认姿态（单位四元数），因为暂时不接收 C 板 IMU 数据
    Eigen::Quaterniond R_gimbal2world(1, 0, 0, 0);
    solver.set_R_gimbal2world(R_gimbal2world);

    // 识别
    std::list<auto_aim::Armor> armors;
    if (use_tradition)
      armors = detector.detect(img, frame_count);
    else
      armors = yolo.detect(img, frame_count);

    // 追踪
    auto targets = tracker.track(armors, timestamp);

    // 瞄准
    // 注意：bullet_speed 暂时设为 15 (m/s)，因为没有从 C 板接收
    double bullet_speed = 15.0; 
    auto command = aimer.aim(targets, timestamp, bullet_speed, false);

    // 开火判断逻辑 (参考 auto_aim_test.cpp)
    if (!targets.empty() && aimer.debug_aim_point.valid &&
        std::abs(command.yaw - last_command.yaw) * 57.3 < 2) {
      command.shoot = true;
    }
    
    // 强制 control 为 true，确保 C 板执行
    command.control = true;

    if (command.control) last_command = command;

    // 3. 发送串口数据
    if (serial_ptr && serial_ptr->isOpen()) {
      uint8_t payload[12];
      io::CBoardComm::encode_command(command, payload);

      std::vector<uint8_t> frame;
      frame.reserve(14);
      frame.push_back(0xFF); // Header
      // No ID
      frame.insert(frame.end(), payload, payload + 12);
      frame.push_back(0x0D); // Tail

      try {
        serial_ptr->write(frame);
        // tools::logger()->info("Sent frame: yaw={:.2f} pitch={:.2f} shoot={}", command.yaw, command.pitch, command.shoot);
      } catch (const std::exception & e) {
        tools::logger()->warn("Serial write failed: {}", e.what());
      }
    }

    // 4. 显示与调试 (可选，为了性能可以注释掉 imshow)
    // tools::draw_text(img, fmt::format("CMD: {:.2f}, {:.2f}, S:{}", command.yaw*57.3, command.pitch*57.3, command.shoot), {10, 30}, {0, 255, 0});
    
    // 绘制装甲板
    for (const auto& armor : armors) {
        tools::draw_points(img, armor.points, {0, 0, 255});
    }
    
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("Simple Serial Aim", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
