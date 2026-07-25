#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "serial/serial.h"
#include <memory>
#include <thread>
#include <atomic>
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  bool force_mode = false;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有
  uint8_t enemy_color = 0; // 0: both, 1: red, 2: blue

  CBoard(const std::string & config_path);
  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能
  mutable tools::ThreadSafeQueue<std::vector<uint8_t>> tx_queue_{100}; // 异步发送队列
  std::unique_ptr<SocketCAN> can_;
  std::unique_ptr<serial::Serial> serial_;
  std::thread serial_thread_;
  std::atomic<bool> serial_quit_{false};
  bool use_serial_{false};
  std::string serial_port_;
  std::string can_interface_;
  int serial_baud_{115200};
  // mutable std::mutex serial_mutex_; // 不再需要，IO操作统一在serial_thread_中执行
  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;

  void callback(const can_frame & frame);
  void handle_frame_bytes(uint32_t id, const uint8_t data[8]);
  void handle_rx_frame(const uint8_t* payload);  // 新协议：处理串口接收帧

  void read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP
