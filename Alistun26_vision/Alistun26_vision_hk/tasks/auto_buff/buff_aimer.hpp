#ifndef AUTO_BUFF__AIMER_HPP
#define AUTO_BUFF__AIMER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>

#include "../auto_aim/planner/planner.hpp"
#include "buff_target.hpp"
#include "buff_type.hpp"
#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"

namespace auto_buff
{
class Aimer
{
public:
  Aimer(const std::string & config_path);

  io::Command aim(
    Target & target, std::chrono::steady_clock::time_point & timestamp, double bullet_speed,
    bool to_now = true);

  auto_aim::Plan mpc_aim(
    Target & target, std::chrono::steady_clock::time_point & timestamp, io::GimbalState gs,
    bool to_now = true);

  double angle;      ///
  double t_gap = 0;  ///

private:
  SmallTarget target_;
  double yaw_offset_;
  double pitch_offset_;
  double yaw_direction_;
  double pitch_direction_;

  double fire_gap_time_;
  double predict_time_;

  int mistake_count_ = 0;
  bool switch_fanblade_;

  double last_yaw_ = 0;
  double last_pitch_ = 0;
  
  // 滤波和平滑
  double smoothed_yaw_ = 0;
  double smoothed_pitch_ = 0;
  double smooth_factor_ = 0.3; // 滤波系数，越小越平滑但滞后越大
  double max_speed_ = 2.0;     // 最大旋转速度 rad/s

  // for mpc
  bool first_in_aimer_ = true;

  std::chrono::steady_clock::time_point last_fire_t_;
  std::chrono::steady_clock::time_point last_aim_time_;

  bool get_send_angle(
    auto_buff::Target & target, const double predict_time, const double bullet_speed,
    const bool to_now, double & yaw, double & pitch);
};
}  // namespace auto_buff
#endif  // AUTO_AIM__AIMER_HPP
