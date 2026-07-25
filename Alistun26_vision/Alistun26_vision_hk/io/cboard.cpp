#include "cboard.hpp"

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include "io/cboard_comm.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/crc.hpp"
#include "serial/serial.h"
#include <memory>
#include <thread>
#include <atomic>
#include <algorithm>
#include <cstring>

namespace io
{
CBoard::CBoard(const std::string & config_path)
: mode(Mode::auto_aim),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(15.0),
  queue_(5000),
  can_(nullptr),
  serial_(nullptr)
{
  // 解析配置（会设置 quaternion_canid_, bullet_speed_canid_, send_canid_，
  // 以及可选的 serial_port_/serial_baud_ 或 can_interface）
  read_yaml(config_path);

  // 如果配置了串口，则优先使用串口
  if (!serial_port_.empty()) {
    try {
      use_serial_ = true;
      tools::logger()->info("[CBoard] Trying to open serial port {} @ {}", serial_port_, serial_baud_);
      serial_.reset(new serial::Serial(serial_port_, static_cast<uint32_t>(serial_baud_), serial::Timeout::simpleTimeout(100)));
      
      if (!serial_->isOpen()) {
          serial_->open();
      }
      
      if (serial_->isOpen()) {
          tools::logger()->info("[CBoard] Successfully opened serial port {} @ {}", serial_port_, serial_baud_);
      } else {
          throw std::runtime_error("Serial port not open after initialization");
      }

      // 启动串口IO线程 (负责读写)
      serial_quit_.store(false);
      serial_thread_ = std::thread([this]() {
        std::vector<uint8_t> buf;
        // 新协议帧长度: Header(1) + w(2) + x(2) + y(2) + z(2) + Speed(2) + EnemyColor(1) + Mode(1) + Ender(1) = 14 字节
        constexpr size_t FRAME_LENGTH = 14;

        while (!serial_quit_.load()) {
          bool busy = false;
          try {
            // --- 1. 处理发送 ---
            if (!tx_queue_.empty()) {
              std::vector<uint8_t> tx_data;
              tx_queue_.pop(tx_data); // 取出一包数据
              if (!tx_data.empty() && serial_ && serial_->isOpen()) {
                 serial_->write(tx_data);
                 busy = true;
              }
            }

            // --- 2. 处理接收 ---
            size_t avail = 0;
            if (serial_ && serial_->isOpen()) {
                avail = serial_->available();
            }
            
            if (avail > 0) {
              busy = true;
              std::vector<uint8_t> data;
              serial_->read(data, avail);

              // DEBUG: Print raw received data
              std::string raw_hex;
              std::string raw_str;
              for (auto b : data) {
                  char hex_buf[4];
                  snprintf(hex_buf, sizeof(hex_buf), "%02X ", b);
                  raw_hex += hex_buf;
                  if (b >= 32 && b <= 126) raw_str += (char)b;
                  else raw_str += '.';
              }
              tools::logger()->info("[CBoard] Recv raw ({} bytes): Hex=[{}] Str=[{}]", data.size(), raw_hex, raw_str);

              buf.insert(buf.end(), data.begin(), data.end());

              // 解析帧
              while (true) {
                auto it = std::find(buf.begin(), buf.end(), 0xFF);
                if (it == buf.end()) { buf.clear(); break; }
                size_t start = std::distance(buf.begin(), it);
                if (buf.size() < start + FRAME_LENGTH) break; 
                if (buf[start + FRAME_LENGTH - 1] != 0x0D) {
                  buf.erase(buf.begin(), buf.begin() + start + 1);
                  continue;
                }
                const uint8_t* payload = buf.data() + start + 1;
                handle_rx_frame(payload);
                buf.erase(buf.begin(), buf.begin() + start + FRAME_LENGTH);
              }
            }

            // 如果既没发送也没接收，稍微休息一下以释放CPU
            if (!busy) {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

          } catch (const std::exception & e) {
            tools::logger()->warn("[CBoard] serial IO error: {}", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
        }
        tools::logger()->info("[CBoard] serial IO thread exiting");
      });
    } catch (const std::exception & e) {
      tools::logger()->warn("[CBoard] Failed to open serial {}: {}", serial_port_, e.what());
      use_serial_ = false;
    }
  }

  // 若未使用串口，则初始化 SocketCAN（保持向后兼容）
  // 仅当明确配置了 CAN 接口且非空时才尝试初始化
  /* CAN disabled by user request
  if (!use_serial_) {
    if (!can_interface_.empty()) {
      try {
        can_.reset(new SocketCAN(can_interface_, std::bind(&CBoard::callback, this, std::placeholders::_1)));
      } catch (const std::exception & e) {
        tools::logger()->warn("[CBoard] Failed to open CAN interface: {}", e.what());
      }
    } else {
      // 既没有成功打开串口，也没有配置 CAN 接口
      tools::logger()->warn("[CBoard] No serial opened and CAN interface not configured/empty. Communication disabled.");
    }
  }
  */

  // tools::logger()->info("[Cboard] Waiting for q...");
  // queue_.pop(data_ahead_);
  // queue_.pop(data_behind_);
  
  // 初始化为单位四元数，避免阻塞启动
  data_ahead_.q = Eigen::Quaterniond::Identity();
  data_ahead_.timestamp = std::chrono::steady_clock::now();
  data_behind_ = data_ahead_;
  
  tools::logger()->info("[Cboard] Opened (Non-blocking).");
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  // 非阻塞检查：如果队列为空，直接返回最近的数据
  if (queue_.empty()) {
    return data_ahead_.q.normalized();
  }

  while (!queue_.empty()) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }
  
  // 如果遍历完队列仍未找到比 timestamp 更晚的数据
  if (data_behind_.timestamp <= timestamp) {
    return data_behind_.q.normalized();
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command) const
{
  // 准备 12 字节 payload (所有模式均使用 12 字节 payload 协议)
  uint8_t payload[12];
  CBoardComm::encode_command(command, payload);

  // Payload: Control(1) Shoot(1) Yaw(4) Pitch(4) Res(2)
  // 限制日志发送频率，降低CPU负载 (20Hz)
  static auto last_send_log = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (tools::delta_time(now, last_send_log) > 0.05) {
    tools::logger()->info("[CBoard] Send Payload (12B): {:02X} {:02X} | {:02X} {:02X} {:02X} {:02X} | {:02X} {:02X} {:02X} {:02X} | {:02X} {:02X}", 
        payload[0], payload[1],
        payload[2], payload[3], payload[4], payload[5], 
        payload[6], payload[7], payload[8], payload[9],
        payload[10], payload[11]);
    
    tools::logger()->info("[CBoard] Input Cmd: ctrl={} shoot={} yaw={:.4f} pitch={:.4f}", 
        command.control, command.shoot, command.yaw, command.pitch);

    last_send_log = now;
  }

  if (use_serial_) {
    // 异步发送：只将数据推入队列，实际发送由 serial_thread_ 处理
    if (serial_ && serial_->isOpen()) {
        // 帧格式: 0xFF payload(12) 0x0D (无ID)
        std::vector<uint8_t> framed;
        framed.reserve(14);
        framed.push_back(0xFF);
        framed.insert(framed.end(), payload, payload + 12);
        framed.push_back(0x0D);
        
        tx_queue_.push(framed);
        return;
    } else {
        tools::logger()->warn("[CBoard] Serial enabled but port not open! Falling back to CAN if available.");
    }
  }

  /* CAN disabled by user request
  if (can_) {
    // Standard CAN max payload is 8 bytes. New protocol is 12 bytes.
    // CAN transmission is disabled/truncated for now as requested by user ("can't use CAN").
    // If CAN FD were available, we could use it, but here we just skip or log warning.
    // tools::logger()->warn("[CBoard] CAN send skipped: Payload (12B) > CAN limit (8B)");
  }
  */
}

void CBoard::callback(const can_frame & frame)
{
  // 转发给统一的字节处理器
  // handle_frame_bytes(frame.can_id, const_cast<uint8_t *>(frame.data));
}

 // 新协议：处理串口接收帧 (12字节 payload)
 // Payload 格式: w(2) + x(2) + y(2) + z(2) + Speed(2) + EnemyColor(1) + Mode(1) = 12 字节
 void CBoard::handle_rx_frame(const uint8_t* payload)
 {
   auto timestamp = std::chrono::steady_clock::now();

   // 解析四元数 (int16, Little Endian, scaled by 10000)
   int16_t w_raw, x_raw, y_raw, z_raw;
   std::memcpy(&w_raw, payload + 0, sizeof(int16_t));
   std::memcpy(&x_raw, payload + 2, sizeof(int16_t));
   std::memcpy(&y_raw, payload + 4, sizeof(int16_t));
   std::memcpy(&z_raw, payload + 6, sizeof(int16_t));

  double x = static_cast<double>(x_raw) / 10000.0;
  double y = static_cast<double>(y_raw) / 10000.0;
  double z = static_cast<double>(z_raw) / 10000.0;
  double w = static_cast<double>(w_raw) / 10000.0;

  // 验证四元数有效性 (放宽检查，因为 int16 精度有限)
  double norm_sq = x * x + y * y + z * z + w * w;
  if (std::abs(norm_sq - 1.0) > 0.1) {
    // 放宽：仅告警，不早退，避免错过后续的 bullet_speed / mode 解析
    tools::logger()->warn("Invalid quaternion: w={} x={} y={} z={} (norm²={})", w, x, y, z, norm_sq);
  } else {
    queue_.push({{w, x, y, z}, timestamp});
  }

  // 解析子弹速度 (int16, Little Endian)。兼容两种缩放：
  // - raw = 18   -> 18 m/s   (单位: m/s)
  // - raw = 1800 -> 18.00 m/s(单位: m/s * 100)
  int16_t bullet_speed_raw;
  std::memcpy(&bullet_speed_raw, payload + 8, sizeof(int16_t));
  // 自动尺度检测：绝对值 >= 1000 认为来自 *100 的缩放，否则按 m/s 解读
  if (bullet_speed_raw >= 1000 || bullet_speed_raw <= -1000) {
    bullet_speed = static_cast<double>(bullet_speed_raw) / 100.0;
  } else {
    bullet_speed = static_cast<double>(bullet_speed_raw);
  }

  // 解析模式 (uint8)
  if (!force_mode) {
    mode = Mode(payload[10]);
  }

  // 解析敌方颜色 (uint8)
  uint8_t new_enemy_color = payload[11];
  if (new_enemy_color != enemy_color) {
    enemy_color = new_enemy_color;
    // 写入文件以通知 tracker 和 decider
    std::string color_str = "both";
    if (enemy_color == 1) color_str = "red";
    else if (enemy_color == 2) color_str = "blue";
    
    // 写入 /tmp/robot_enemy_color
    FILE* fp = fopen("/tmp/robot_enemy_color", "w");
    if (fp) {
      fprintf(fp, "%s\n", color_str.c_str());
      fclose(fp);
      tools::logger()->info("[CBoard] Enemy color updated to: {}", color_str);
    }
  }

  // 限制日志输出频率为1Hz
  static auto last_log_time = std::chrono::steady_clock::time_point::min();
  auto now = std::chrono::steady_clock::now();

  if (tools::delta_time(now, last_log_time) >= 1.0) {
    tools::logger()->info(
      "[CBoard] Quaternion: w={:.4f} x={:.4f} y={:.4f} z={:.4f}, Bullet speed: {:.2f} m/s, EnemyColor: {}, Mode: {}",
      w, x, y, z, bullet_speed, enemy_color, MODES[mode]);
    last_log_time = now;
  }
}

// 旧协议：处理 CAN 帧 (8字节 payload) - 保留用于向后兼容
void CBoard::handle_frame_bytes(uint32_t id, const uint8_t data[8])
{
  /* CAN disabled by user request
  auto timestamp = std::chrono::steady_clock::now();

  // Support both full CAN ID and 1-byte Serial ID (LSB)
  if ((id & 0xFF) == (quaternion_canid_ & 0xFF)) {
    auto x = static_cast<double>(static_cast<int16_t>((data[0] << 8) | data[1])) / 1e4;
    auto y = static_cast<double>(static_cast<int16_t>((data[2] << 8) | data[3])) / 1e4;
    auto z = static_cast<double>(static_cast<int16_t>((data[4] << 8) | data[5])) / 1e4;
    auto w = static_cast<double>(static_cast<int16_t>((data[6] << 8) | data[7])) / 1e4;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({{w, x, y, z}, timestamp});
  } else if ((id & 0xFF) == (bullet_speed_canid_ & 0xFF)) {
    bullet_speed = static_cast<double>(static_cast<int16_t>((data[0] << 8) | data[1])) / 1e2;
    mode = Mode(data[2]);
    shoot_mode = ShootMode(data[3]);
    ft_angle = static_cast<double>(static_cast<int16_t>((data[4] << 8) | data[5])) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }
  */
}

// 实现方式有待改进
void CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  // 优先读取串口配置（可选）
  if (yaml["serial_port"]) {
    serial_port_ = yaml["serial_port"].as<std::string>();
    if (yaml["serial_baud"]) serial_baud_ = yaml["serial_baud"].as<int>();
  }

  // 读取 CAN 接口（可选，作为串口未配置时的回退）
  if (yaml["can_interface"]) {
    can_interface_ = yaml["can_interface"].as<std::string>();
  }
}

CBoard::~CBoard()
{
  // 停止串口线程
  try {
    serial_quit_.store(true);
    if (serial_thread_.joinable()) serial_thread_.join();
    if (serial_ && serial_->isOpen()) serial_->close();
  } catch (...) {
  }
}

}  // namespace io
