#include "buff_aimer.hpp"

#include <memory>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_buff
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["buff_yaw_offset"] ? yaml["buff_yaw_offset"].as<double>() / 57.3 : yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["buff_pitch_offset"] ? yaml["buff_pitch_offset"].as<double>() / 57.3 : yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  yaw_direction_ = yaml["yaw_direction"] ? yaml["yaw_direction"].as<double>() : 1.0;
  pitch_direction_ = yaml["pitch_direction"] ? yaml["pitch_direction"].as<double>() : 1.0;
  fire_gap_time_ = yaml["fire_gap_time"].as<double>();
  predict_time_ = yaml["predict_time"].as<double>();
  smooth_factor_ = yaml["smooth_factor"] ? yaml["smooth_factor"].as<double>() : 0.3;
  max_speed_ = yaml["max_speed"] ? yaml["max_speed"].as<double>() : 2.0;

  last_fire_t_ = std::chrono::steady_clock::now();
  last_aim_time_ = std::chrono::steady_clock::now();
}

io::Command Aimer::aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now)
{
  io::Command command = {false, false, 0, 0};
  if (target.is_unsolve()) return command;

  // 如果子弹速度异常偏小（如串口缩放错误导致），回退到 15 m/s
  if (bullet_speed < 0.1) bullet_speed = 15.0;

  auto now = std::chrono::steady_clock::now();

  auto detect_now_gap = tools::delta_time(now, timestamp);
  auto future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_;
  double yaw, pitch;

  bool angle_changed =
    std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3;
  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
    // 1. 滤波 (低通滤波)
    // 如果是第一次运行或重置状态（last_yaw_为0），直接赋值
    if (std::abs(last_yaw_) < 1e-6 && std::abs(last_pitch_) < 1e-6) {
        smoothed_yaw_ = yaw;
        smoothed_pitch_ = pitch;
    } else {
        smoothed_yaw_ = smoothed_yaw_ * (1 - smooth_factor_) + yaw * smooth_factor_;
        smoothed_pitch_ = smoothed_pitch_ * (1 - smooth_factor_) + pitch * smooth_factor_;
    }

    // 2. 限速
    // 使用实际帧间隔计算最大变化量
    double dt = tools::delta_time(now, last_aim_time_);
    if (dt > 0.1) dt = 0.01; // 如果间隔过大（如丢帧），限制为 10ms，避免跳变
    if (dt < 0.001) dt = 0.001; // 避免除零

    double max_diff = max_speed_ * dt; 
    double dy = smoothed_yaw_ - last_yaw_;
    double dp = smoothed_pitch_ - last_pitch_;

    if (std::abs(dy) > max_diff) {
        dy = (dy > 0 ? 1 : -1) * max_diff;
        smoothed_yaw_ = last_yaw_ + dy;
    }
    if (std::abs(dp) > max_diff) {
        dp = (dp > 0 ? 1 : -1) * max_diff;
        smoothed_pitch_ = last_pitch_ + dp;
    }

    // 使用平滑后的值
    command.yaw = smoothed_yaw_ * yaw_direction_;
    command.pitch = -smoothed_pitch_ * pitch_direction_; // 恢复负号

    // 判断是否切换扇叶或目标突变
    // 使用平滑后的值与上一帧比较，或者使用原始值与上一帧比较？
    // 这里使用原始值 yaw 与 last_yaw_ 比较可能更敏感，能及时检测突变
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;
      mistake_count_ = 0;
      command.control = true;
      // 重置平滑器，避免拖尾
      smoothed_yaw_ = yaw;
      smoothed_pitch_ = pitch;
    } else if (std::abs(last_yaw_ - smoothed_yaw_) > 5 / 57.3 || std::abs(last_pitch_ - smoothed_pitch_) > 5 / 57.3) {
      switch_fanblade_ = true;
      mistake_count_++;
      command.control = false;
    } else {
      switch_fanblade_ = false;
      mistake_count_ = 0;
      command.control = true;
    }
    last_yaw_ = smoothed_yaw_;
    last_pitch_ = smoothed_pitch_;
    last_aim_time_ = now;
  }

  if (switch_fanblade_) {
    command.shoot = false;
    last_fire_t_ = now;
  } else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    command.shoot = true;
    last_fire_t_ = now;
  }

  return command;
}

auto_aim::Plan Aimer::mpc_aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp, io::GimbalState gs,
  bool to_now)
{
  auto_aim::Plan plan = {false, false, 0, 0, 0, 0, 0, 0, 0, 0};
  if (target.is_unsolve()) return plan;

  double bullet_speed;
  // 如果子弹速度异常偏小（如串口缩放错误导致），回退到 15 m/s
  if (gs.bullet_speed < 1.0)
    bullet_speed = 15.0;
  else
    bullet_speed = gs.bullet_speed;

  auto now = std::chrono::steady_clock::now();

  auto detect_now_gap = tools::delta_time(now, timestamp);
  auto future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_;
  double yaw, pitch;

  bool angle_changed =
    std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3;
  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
    plan.yaw = yaw * yaw_direction_;
    plan.pitch = -pitch * pitch_direction_;
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;
      mistake_count_ = 0;
      plan.control = true;
      first_in_aimer_ = true;
    } else if (std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3) {
      switch_fanblade_ = true;
      mistake_count_++;
      plan.control = false;

      first_in_aimer_ = true;
    } else {
      switch_fanblade_ = false;
      mistake_count_ = 0;
      plan.control = true;
    }
    last_yaw_ = yaw;
    last_pitch_ = pitch;

    if (plan.control) {
      if (first_in_aimer_) {
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
        first_in_aimer_ = false;
      } else {
        auto dt = predict_time_;
        double last_yaw_mpc, last_pitch_mpc;
        
        // 修复：为了计算速度，需要向后预测。但不能直接修改 target，否则会抵消掉前一次的预测量，
        // 导致 mt_standard.cpp 中获取到的 target_copy 丢失了 predict_time 的提前量，画出来的预测框永远滞后。
        std::unique_ptr<Target> temp_target;
        if (auto p = dynamic_cast<SmallTarget*>(&target)) {
          temp_target = std::make_unique<SmallTarget>(*p);
        } else if (auto p = dynamic_cast<BigTarget*>(&target)) {
          temp_target = std::make_unique<BigTarget>(*p);
        }

        if (temp_target) {
            get_send_angle(
              *temp_target, predict_time_ * -1, bullet_speed, to_now, last_yaw_mpc, last_pitch_mpc);
        } else {
            last_yaw_mpc = yaw;
            last_pitch_mpc = pitch;
        }

        plan.yaw_vel = tools::limit_rad(yaw - last_yaw_mpc) / (2 * dt);
        // plan.yaw_vel = tools::limit_min_max(plan.yaw_vel, -6.28, 6.28);
        // 由于没有实时的 IMU 数据，去掉基于 gs.yaw 计算的“速度反馈”项，防止加速度爆炸
        plan.yaw_acc = tools::limit_rad(yaw - gs.yaw) / std::pow(dt, 2);
        // plan.yaw_acc = tools::limit_min_max(plan.yaw_acc, -50, 50);

        plan.pitch_vel = tools::limit_rad(-pitch + last_pitch_mpc) / (2 * dt);
        // plan.pitch_vel = tools::limit_min_max(plan.pitch_vel, -6.28, 6.28);
        plan.pitch_acc = tools::limit_rad(-pitch - gs.pitch) / std::pow(dt, 2);
        // plan.pitch_acc = tools::limit_min_max(plan.pitch_acc, -100, 100);
      }
    }
  }

  if (switch_fanblade_) {
    plan.fire = false;
    last_fire_t_ = now;
  } else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    plan.fire = true;
    last_fire_t_ = now;
  }

  return plan;
}

bool Aimer::get_send_angle(
  auto_buff::Target & target, const double predict_time, const double bullet_speed,
  const bool to_now, double & yaw, double & pitch)
{
  // 考虑detecor所消耗的时间，此外假设aimer的用时可忽略不计
  // 如果 to_now 为 true，则根据当前时间和时间戳预测目标位置,deltatime = 现在时间减去当时照片时间，加上0.1
  double spd = target.ekf_x()[6];
  bool is_rotating = std::abs(spd) > 0.1;

  auto current_pos = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.9));

  if (is_rotating) target.predict(predict_time);
  // std::cout << "gap: " << detect_now_gap << std::endl;
  angle = target.ekf_x()[5];

  // 计算目标点的空间坐标
  auto aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.9));
  double d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  double h = aim_in_world[2];

  // 创建初始弹道对象
  tools::Trajectory current_traj(bullet_speed, d, h);
  if (current_traj.unsolvable) {  // 如果弹道无法解算，使用几何回退继续跟踪（不开火）
    tools::logger()->debug(
      "[Aimer] Unsolvable initial trajectory: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    // 几何回退：不进行重力补偿，仅用相对方位角/仰角保持跟踪
    double d_safe = (d < 1e-6) ? 1e-6 : d;
    yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
    pitch = std::atan2(h, d_safe) + pitch_offset_;
    return true;
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  double prev_fly_time = current_traj.fly_time;
  bool converged = false;

  for (int iter = 0; iter < 10; ++iter) {
    // 基于发射时刻的状态(target)，预测飞行时间后的状态
    // 由于Target是抽象类，我们需要根据具体类型创建副本
    std::unique_ptr<Target> iter_target;
    if (auto p = dynamic_cast<SmallTarget*>(&target)) {
      iter_target = std::make_unique<SmallTarget>(*p);
    } else if (auto p = dynamic_cast<BigTarget*>(&target)) {
      iter_target = std::make_unique<BigTarget>(*p);
    } else {
      // 未知类型，无法复制，直接退出迭代
      break;
    }

    if (is_rotating) iter_target->predict(prev_fly_time);
    
    // 更新目标点
    aim_in_world = iter_target->point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.9));
    d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
    h = aim_in_world[2];

    // 计算新弹道
    current_traj = tools::Trajectory(bullet_speed, d, h);

    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, h={:.2f}", iter + 1,
        bullet_speed, d, h);
      // 几何回退
      double d_safe = (d < 1e-6) ? 1e-6 : d;
      yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
      pitch = std::atan2(h, d_safe) + pitch_offset_;
      return true;
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 更新最终的目标状态用于外部使用（如果需要）
  // target目前处于发射时刻状态，再预测飞行时间即可
  if (is_rotating) target.predict(prev_fly_time);
  angle = target.ekf_x()[5];

  // 计算偏航角和俯仰角，并返回命中结果
  yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
  pitch = current_traj.pitch + pitch_offset_;

  tools::logger()->info("Buff Pred: PredictTimeConf:{:.3f} TotalFuture:{:.3f} Spd:{:.3f} Time:{:.3f} Cur:({:.3f},{:.3f},{:.3f}) Pred:({:.3f},{:.3f},{:.3f}) Diff:({:.3f},{:.3f},{:.3f})",
      predict_time_, predict_time,
      spd, prev_fly_time,
      current_pos[0], current_pos[1], current_pos[2],
      aim_in_world[0], aim_in_world[1], aim_in_world[2],
      aim_in_world[0] - current_pos[0],
      aim_in_world[1] - current_pos[1],
      aim_in_world[2] - current_pos[2]);

  return true;
};

}  // namespace auto_buff
