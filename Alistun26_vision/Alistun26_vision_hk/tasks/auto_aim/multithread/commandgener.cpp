#include "commandgener.hpp"

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace multithread
{

CommandGener::CommandGener(
  auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
  tools::Plotter & plotter, bool debug)
: shooter_(shooter), aimer_(aimer), cboard_(cboard), plotter_(plotter), stop_(false), debug_(debug)
{
  thread_ = std::thread(&CommandGener::generate_command, this);
}

CommandGener::~CommandGener()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed, const Eigen::Vector3d & gimbal_pos, bool allow_shoot)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {targets, t, bullet_speed, gimbal_pos, allow_shoot};
  cv_.notify_one();
}

io::Command CommandGener::get_latest_command()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return latest_cmd_;
}

auto_aim::AimPoint CommandGener::get_debug_aim_point()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return aimer_.debug_aim_point;
}

void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();
  while (!stop_) {
    std::optional<Input> input;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.2) {
        input = latest_;
      } else
        input = std::nullopt;
    }
    if (input) {
      auto command = aimer_.aim(input->targets_, input->t, input->bullet_speed);
      command.shoot = shooter_.shoot(command, aimer_, input->targets_, input->gimbal_pos);
      
      if (!input->allow_shoot) {
        command.shoot = false;
      }

      command.horizon_distance = input->targets_.empty()
                                   ? 0
                                   : std::sqrt(
                                       tools::square(input->targets_.front().ekf_x()[0]) +
                                       tools::square(input->targets_.front().ekf_x()[2]));
      
      {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_cmd_ = command;
      }
      
      cboard_.send(command);
      if (debug_) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
        data["shoot"] = command.shoot;
        data["horizon_distance"] = command.horizon_distance;
        plotter_.plot(data);
      }
    } else {
      // 如果没有目标或数据过旧，发送空命令以保持通信
      // 但仅当处于自瞄模式时才发送，避免干扰其他模式（如打符）
      if (cboard_.mode == io::Mode::auto_aim || cboard_.mode == io::Mode::outpost) {
        io::Command empty_command;
        empty_command.yaw = 0;
        empty_command.pitch = 0;
        empty_command.shoot = false;
        empty_command.control = false; // 表示不控制云台
        cboard_.send(empty_command);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));  //approximately 500Hz
  }
}

}  // namespace multithread

}  // namespace auto_aim
