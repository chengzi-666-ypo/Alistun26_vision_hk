#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include <fstream>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include <fmt/core.h>

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

// 辅助函数：更新 YAML 配置文件中的偏移量并保留注释
void update_yaml_offsets(const std::string & filepath, double new_yaw, double new_pitch)
{
  std::ifstream in(filepath);
  if (!in.is_open()) return;
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line)) {
    // 匹配 aimer 参数中的 yaw_offset 和 pitch_offset
    if (
      line.find("yaw_offset:") != std::string::npos &&
      line.find("buff_yaw_offset") == std::string::npos &&
      line.find("left_yaw_offset") == std::string::npos &&
      line.find("right_yaw_offset") == std::string::npos) {
      size_t comment_pos = line.find('#');
      std::string comment = (comment_pos != std::string::npos) ? " " + line.substr(comment_pos) : "";
      line = fmt::format("yaw_offset: {:.2f}{}", new_yaw, comment);
    } else if (
      line.find("pitch_offset:") != std::string::npos &&
      line.find("buff_pitch_offset") == std::string::npos) {
      size_t comment_pos = line.find('#');
      std::string comment = (comment_pos != std::string::npos) ? " " + line.substr(comment_pos) : "";
      line = fmt::format("pitch_offset: {:.2f}{}", new_pitch, comment);
    }
    lines.push_back(line);
  }
  in.close();
  std::ofstream out(filepath);
  for (const auto & l : lines) {
    out << l << "\n";
  }
  out.close();
}

struct BuffResult
{
  cv::Mat img;
  std::optional<auto_buff::PowerRune> power_runes;
  std::chrono::steady_clock::time_point t;
};

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  tools::ThreadSafeQueue<BuffResult, true> buff_queue(1);

  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter);

  std::atomic<io::Mode> mode{io::Mode::idle};
  auto last_mode{io::Mode::idle};

  // 拨杆标定自适应偏移量相关变量
  std::vector<std::chrono::steady_clock::time_point> mode_switch_times;
  bool trigger_calibration = false;
  int calib_wait_frames = 0;

  // FPS calculation variables
  int fps = 0;
  int frame_count = 0;
  auto last_fps_time = std::chrono::steady_clock::now();

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      if (mode.load() == io::Mode::auto_aim || mode.load() == io::Mode::outpost) {
        camera.read(img, t);
        if (img.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        detector.push(img, t);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      } else if (mode.load() == io::Mode::small_buff || mode.load() == io::Mode::big_buff) {
        camera.read(img, t);
        if (img.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        buff_detector.push(img, t);
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
    }
  });

  auto buff_result_thread = std::thread([&]() {
    while (!exiter.exit()) {
      if (mode.load() == io::Mode::small_buff || mode.load() == io::Mode::big_buff) {
        auto [power_runes, t, img] = buff_detector.pop();
        BuffResult result{img, power_runes, t};
        buff_queue.push(result);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  });

  while (!exiter.exit()) {
    mode = cboard.mode;
    
    // 如果需要强制调试打符模式，请取消下面两行的注释：
     mode = io::Mode::auto_aim;
     //mode = io::Mode::small_buff;
     cboard.force_mode = true;
    
    // 同步更新 cboard.mode 以防止 CommandGener 线程发送空命令干扰
    cboard.mode = mode;

    if (last_mode != mode) {
      auto now_time = std::chrono::steady_clock::now();
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      
      // 检测拨杆连续切换以触发自适应标定
      if (mode == io::Mode::auto_aim) {
          mode_switch_times.push_back(now_time);
          // 清理超过 3 秒的记录
          while (!mode_switch_times.empty() && 
                 std::chrono::duration_cast<std::chrono::seconds>(now_time - mode_switch_times.front()).count() > 3) {
              mode_switch_times.erase(mode_switch_times.begin());
          }
          // 如果 3 秒内连续切换到 auto_aim 3 次（即 关-开-关-开-关-开）
          if (mode_switch_times.size() >= 3) {
              tools::logger()->warn("AUTO CALIBRATION TRIGGERED BY SWITCH TOGGLE!");
              mode_switch_times.clear();
              trigger_calibration = true;
              calib_wait_frames = 0;
          }
      }
      
      last_mode = mode.load();
    }

    /// 自瞄
    if (mode.load() == io::Mode::auto_aim || mode.load() == io::Mode::outpost) {
      // Calculate FPS
      frame_count++;
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_fps_time).count() >= 1) {
        fps = frame_count;
        frame_count = 0;
        last_fps_time = now;
      }

      auto [img, armors, t] = detector.debug_pop();

      // 根据模式过滤装甲板
      if (mode.load() == io::Mode::auto_aim) {
        // auto_aim模式只识别1~5号和sentry装甲板
        armors.erase(
          std::remove_if(
            armors.begin(), armors.end(),
            [](const auto_aim::Armor & armor) {
              return armor.name == auto_aim::ArmorName::outpost ||
                     armor.name == auto_aim::ArmorName::base ||
                     armor.name == auto_aim::ArmorName::not_armor;
            }),
          armors.end());
      } else if (mode.load() == io::Mode::outpost) {
        // outpost模式只识别outpost装甲板
        armors.erase(
          std::remove_if(
            armors.begin(), armors.end(),
            [](const auto_aim::Armor & armor) {
              return armor.name != auto_aim::ArmorName::outpost;
            }),
          armors.end());
      }
      Eigen::Quaterniond q = cboard.imu_at(t - 1ms);

      // recorder.record(img, q, t);

      solver.set_R_gimbal2world(q);

      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      auto targets = tracker.track(armors, t);

      // 判断是否允许开火：只有在 TRACKING 状态（即有视觉检测）时才允许开火
      // 如果是 TEMP_LOST（不亮灯/预测中），则不允许开火
      bool allow_shoot = (tracker.state() == "tracking");
      
      // 执行拨杆自适应校准逻辑
      if (trigger_calibration) {
          if (allow_shoot && !targets.empty()) {
              // 暂时将 offset 设为 0 以获取无偏移时的基础预测角度
              aimer.set_yaw_offset(0.0);
              aimer.set_pitch_offset(0.0);
              
              auto calib_cmd = aimer.aim(targets, t, cboard.bullet_speed, true);
              
              double base_yaw = calib_cmd.yaw;
              double base_pitch = -calib_cmd.pitch; // 恢复原始 pitch 正负
              
              double current_gimbal_yaw = ypr[0];
              double current_gimbal_pitch = ypr[1];
              
              // 计算新的 offset（弧度），使预测输出正好等于当前云台角度
              double new_yaw_offset_rad = current_gimbal_yaw - base_yaw;
              double new_pitch_offset_rad = -current_gimbal_pitch - base_pitch;
              
              // 转换为度数并限制范围以防异常
              double new_yaw_offset_deg = tools::limit_min_max(new_yaw_offset_rad * 57.3, -20.0, 20.0);
              double new_pitch_offset_deg = tools::limit_min_max(new_pitch_offset_rad * 57.3, -20.0, 20.0);
              
              aimer.set_yaw_offset(new_yaw_offset_deg);
              aimer.set_pitch_offset(new_pitch_offset_deg);
              
              // 写入 yaml 文件永久保存
              update_yaml_offsets(config_path, new_yaw_offset_deg, new_pitch_offset_deg);
              
              tools::logger()->info("AUTO CALIBRATION SUCCESS! new_yaw_offset: {:.2f}, new_pitch_offset: {:.2f}", 
                                    new_yaw_offset_deg, new_pitch_offset_deg);
                                    
              trigger_calibration = false;
          } else {
              calib_wait_frames++;
              if (calib_wait_frames > 50) { // 最多等待约 1 秒让追踪器锁定
                  tools::logger()->warn("AUTO CALIBRATION FAILED: No target tracking after toggle.");
                  trigger_calibration = false;
                  calib_wait_frames = 0;
              }
          }
      }

      commandgener.push(targets, t, cboard.bullet_speed, ypr, allow_shoot);  // 发送给决策线程

      // 获取最新命令状态用于显示
      auto cmd = commandgener.get_latest_command();

      // --- 可视化部分 ---
      if (!img.empty()) {
          // 创建检测窗口图像副本
          cv::Mat img_detect = img.clone();

          // 1. 窗口一：Detection (显示识别到的装甲板)
          for (const auto& armor : armors) {
              tools::draw_points(img_detect, armor.points, {0, 255, 0}); // 绿色
              
              // 显示 ID 和 颜色
              std::string id_str = "Unknown";
              if (armor.name <= auto_aim::ArmorName::five) {
                  id_str = std::to_string(armor.name + 1);
              } else if (armor.name == auto_aim::ArmorName::sentry) {
                  id_str = "Sentry";
              } else if (armor.name == auto_aim::ArmorName::outpost) {
                  id_str = "Outpost";
              } else if (armor.name == auto_aim::ArmorName::base) {
                  id_str = "Base";
              }
              std::string color_str = (armor.color == auto_aim::Color::blue) ? "B" : "R";
              
              // 在装甲板第一个点附近绘制
              if (!armor.points.empty()) {
                  cv::putText(img_detect, fmt::format("{}{}", color_str, id_str), armor.points[0], cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
              }
          }
          
          // 在检测窗口显示装甲板信息
          if (!armors.empty()) {
              const auto & armor = armors.front();
              tools::draw_text(
                img_detect,
                fmt::format("Armor: x={:.2f}, y={:.2f}, z={:.2f}", 
                            armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2]),
                {10, 30}, {0, 255, 255});
              tools::draw_text(
                img_detect,
                fmt::format("Armor yaw: world={:.2f}, raw={:.2f}", 
                            armor.ypr_in_world[0] * 57.3, armor.yaw_raw * 57.3),
                {10, 60}, {0, 255, 255});
          }

          // 2. 窗口二：Prediction (显示预测和跟踪信息)
          
          // 绘制检测到的装甲板（绿色框）- 方便对比
          for (const auto& armor : armors) {
              tools::draw_points(img, armor.points, {0, 255, 0}); // 绿色
          }

          // 绘制跟踪目标
          for (const auto& target : targets) {
              std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
              for (const Eigen::Vector4d & xyza : armor_xyza_list) {
                  auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
                  // Safety check for drawing points
                  bool points_valid = true;
                  for (const auto& p : image_points) {
                      if (std::isnan(p.x) || std::isnan(p.y) || 
                          std::abs(p.x) > 10000 || std::abs(p.y) > 10000) {
                          points_valid = false;
                          break;
                      }
                  }
                  if (points_valid && !image_points.empty()) {
                      tools::draw_points(img, image_points, {0, 255, 255}); // 黄色
                  }
              }
              
              // 显示目标ID
              if (!target.armor_xyza_list().empty()) {
                  Eigen::VectorXd x = target.ekf_x();
                  std::vector<cv::Point3f> world_points = {cv::Point3f(x[0], x[2], x[4])};
                  auto image_points = solver.world2pixel(world_points);
                  if (!image_points.empty()) {
                      auto& p = image_points[0];
                      if (!std::isnan(p.x) && !std::isnan(p.y) && 
                          std::abs(p.x) < 10000 && std::abs(p.y) < 10000) {
                          
                          std::string id_str = "Unknown";
                          if (target.name <= auto_aim::ArmorName::five) {
                              id_str = std::to_string(target.name + 1);
                          } else if (target.name == auto_aim::ArmorName::sentry) {
                              id_str = "Sentry";
                          } else if (target.name == auto_aim::ArmorName::outpost) {
                              id_str = "Outpost";
                          } else if (target.name == auto_aim::ArmorName::base) {
                              id_str = "Base";
                          }
                          cv::putText(img, fmt::format("ID:{}", id_str), image_points[0], cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2);
                      }
                  }
              }
          }

          // 绘制预测打击点
          if (!targets.empty()) {
              auto aim_point = commandgener.get_debug_aim_point();
              if (aim_point.valid) {
                  std::vector<cv::Point3f> aim_points_3d = {cv::Point3f(aim_point.xyza.x(), aim_point.xyza.y(), aim_point.xyza.z())};
                  auto image_points = solver.world2pixel(aim_points_3d);
                  if (!image_points.empty()) {
                      auto& p = image_points[0];
                      if (!std::isnan(p.x) && !std::isnan(p.y) && 
                          std::abs(p.x) < 10000 && std::abs(p.y) < 10000) {
                          cv::circle(img, image_points[0], 3, {0, 0, 255}, -1); // 实心小红点
                      }
                  }
              }
          }

          // 显示基本信息 (Prediction 窗口)
          tools::draw_text(img, fmt::format("Mode: {}", io::MODES[mode]), {10, 30}, {255, 255, 255});
          // 显示 FPS
          tools::draw_text(img, fmt::format("FPS: {}", fps), {10, 120}, {255, 255, 255});
          
          // 如果子弹速度小于10，显示为15
          double display_speed = (cboard.bullet_speed < 5) ? 15.0 : cboard.bullet_speed;
          tools::draw_text(img, fmt::format("Bullet Speed: {:.1f}", display_speed), {10, 90}, {255, 255, 255});
          
          // 显示命令角度信息
          tools::draw_text(
            img,
            fmt::format("Cmd: ctrl={}, yaw={:.2f}, pitch={:.2f}, shoot={}", 
                        cmd.control, cmd.yaw * 57.3, cmd.pitch * 57.3, cmd.shoot),
            {10, 60}, {154, 50, 205});
          
          // 显示云台当前姿态
          tools::draw_text(
            img,
            fmt::format("Gimbal YPR: yaw={:.2f}, pitch={:.2f}, roll={:.2f}", 
                        ypr[0] * 57.3, ypr[1] * 57.3, ypr[2] * 57.3),
            {10, 210}, {255, 255, 255});
          
          // 显示跟踪状态和颜色
          std::string color_str = "None";
          if (!armors.empty()) {
              color_str = (armors.front().color == auto_aim::Color::blue) ? "Blue" : "Red";
          } else if (!targets.empty()) {
              color_str = "Predicting";
          }
          tools::draw_text(img, fmt::format("State: {}", tracker.state()), {10, 150}, {255, 255, 0});
          tools::draw_text(img, fmt::format("Color: {}", color_str), {10, 180}, {255, 255, 0});

          // 显示当前偏移量信息
          tools::draw_text(
            img,
            fmt::format("Offset: yaw={:.2f}, pitch={:.2f} (Keys: WASD to adjust, C to save)", 
                        aimer.get_yaw_offset(), aimer.get_pitch_offset()),
            {10, 240}, {0, 255, 0});

          // 显示开火状态
          if (cmd.shoot) {
              // 右上角显示 FIRE
              tools::draw_text(img, "FIRE", {img.cols - 200, 100}, {0, 0, 255}, 3.0);
          }

          // 显示两个窗口
          // cv::resize(img_detect, img_detect, {}, 0.5, 0.5);
          // cv::imshow("Detection", img_detect);

          // cv::resize(img, img, {}, 0.5, 0.5);
          // cv::imshow("Prediction", img);

          // int key = cv::waitKey(1);
          int key = -1; // 禁用显示和按键输入
          if (key == 'w' || key == 'W') {
            aimer.set_pitch_offset(aimer.get_pitch_offset() + 0.05);
          } else if (key == 's' || key == 'S') {
            aimer.set_pitch_offset(aimer.get_pitch_offset() - 0.05);
          } else if (key == 'a' || key == 'A') {
            aimer.set_yaw_offset(aimer.get_yaw_offset() + 0.05);
          } else if (key == 'd' || key == 'D') {
            aimer.set_yaw_offset(aimer.get_yaw_offset() - 0.05);
          } else if (key == 'c' || key == 'C') {
            update_yaml_offsets(config_path, aimer.get_yaw_offset(), aimer.get_pitch_offset());
            tools::logger()->info("Offsets saved to config file: yaw={:.2f}, pitch={:.2f}", 
                                  aimer.get_yaw_offset(), aimer.get_pitch_offset());
          }
      }
      // ------------------

    }

    /// 打符
    else if (mode.load() == io::Mode::small_buff || mode.load() == io::Mode::big_buff) {
      // Calculate FPS for Buff mode
      auto now_fps = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now_fps - last_fps_time).count() >= 1) {
        fps = frame_count;
        tools::logger()->info("Buff FPS: {}", fps);
        frame_count = 0;
        last_fps_time = now_fps;
      }

      static BuffResult last_result;
      static bool has_data = false;
      bool new_data = false;

      // 尝试从异步检测器获取数据 (非阻塞)
      if (!buff_queue.empty()) {
        last_result = buff_queue.pop();
        
        has_data = true;
        new_data = true;
        frame_count++; 

        Eigen::Quaterniond q = cboard.imu_at(last_result.t - 1ms);

        buff_solver.set_R_gimbal2world(q);
        buff_solver.solve(last_result.power_runes);

        if (mode.load() == io::Mode::small_buff) {
          buff_small_target.get_target(last_result.power_runes, last_result.t);
        } else if (mode.load() == io::Mode::big_buff) {
          buff_big_target.get_target(last_result.power_runes, last_result.t);
        }
      }

      // 检查数据是否过时 (超过 500ms)
      if (has_data) {
        auto now = std::chrono::steady_clock::now();
        if (tools::delta_time(now, last_result.t) > 0.5) {
          has_data = false;
          tools::logger()->warn("Buff data timeout, stop aiming.");
        }
      }

      // 无论是否有新数据，都进行预测和发送（基于EKF）
      io::Command buff_command;
      Eigen::Vector3d pred_xyz(0, 0, 0); // 保存预测的物理坐标
      std::optional<auto_buff::SmallTarget> pred_small_target;
      std::optional<auto_buff::BigTarget> pred_big_target;

      if (has_data) {
        io::GimbalState gs;
        // 关键修复：必须使用当前最新的云台姿态进行 MPC 预测，不能使用上一帧图像时刻的姿态！
        // 因为 mpc_aim 计算 yaw_acc 时依赖实时的 gs.yaw。如果不更新，加速度会随着 target 预测的推进而爆炸！
        auto now_t = std::chrono::steady_clock::now();
        Eigen::Vector3d gimbal_ypr = tools::eulers(cboard.imu_at(now_t), 2, 1, 0);
        gs.yaw = gimbal_ypr[0];
        gs.pitch = gimbal_ypr[1];
        gs.yaw_vel = 0;
        gs.pitch_vel = 0;
        gs.bullet_speed = cboard.bullet_speed;

        auto_aim::Plan buff_plan;

        if (mode.load() == io::Mode::small_buff) {
          auto target_copy = buff_small_target;
          buff_plan = buff_aimer.mpc_aim(target_copy, last_result.t, gs, true);
          pred_small_target.emplace(target_copy);
          if (!target_copy.is_unsolve()) pred_xyz = target_copy.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.9));
        } else if (mode.load() == io::Mode::big_buff) {
          auto target_copy = buff_big_target;
          buff_plan = buff_aimer.mpc_aim(target_copy, last_result.t, gs, true);
          pred_big_target.emplace(target_copy);
          if (!target_copy.is_unsolve()) pred_xyz = target_copy.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.9));
        }

        buff_command.yaw = buff_plan.yaw;
        buff_command.pitch = buff_plan.pitch;
        buff_command.control = buff_plan.control;
        buff_command.shoot = buff_plan.fire;
        buff_command.horizon_distance = 0;
        
        // Populate MPC fields
        buff_command.yaw_vel = buff_plan.yaw_vel;
        buff_command.yaw_acc = buff_plan.yaw_acc;
        buff_command.pitch_vel = buff_plan.pitch_vel;
        buff_command.pitch_acc = buff_plan.pitch_acc;

        // Debug Log
        if (has_data && last_result.power_runes.has_value()) {
            Eigen::Vector3d gimbal_ypr = tools::eulers(buff_solver.R_gimbal2world(), 2, 1, 0);
            tools::logger()->info(
                "[Buff] Cmd: Y={:.3f} P={:.3f} | Gimbal: Y={:.3f} P={:.3f} | Diff: Y={:.3f} P={:.3f}",
                buff_command.yaw, buff_command.pitch,
                gimbal_ypr[0], gimbal_ypr[1],
                buff_command.yaw - gimbal_ypr[0], buff_command.pitch - gimbal_ypr[1]
            );
        }

        cboard.send(buff_command);

        // --- 可视化部分 ---
        // 响应用户请求，移除显示频率限制
        if (new_data && !last_result.img.empty()) {
          cv::Mat img_debug = last_result.img.clone();
          auto & power_runes = last_result.power_runes;

          // --- 通用信息显示 (无论是否检测到目标都显示) ---
          // 显示信息
          tools::draw_text(
            img_debug, fmt::format("Mode: {}", io::MODES[mode]), {10, 30}, {255, 255, 255});

          // 显示 Bullet Speed
          double display_speed = (cboard.bullet_speed < 5) ? 15.0 : cboard.bullet_speed;
          tools::draw_text(img_debug, fmt::format("Bullet Speed: {:.1f}", display_speed), {10, 90}, {255, 255, 255});

          // 显示 FPS (黄色)
          tools::draw_text(img_debug, fmt::format("FPS: {}", fps), {10, 120}, {0, 255, 255});

          // 显示云台当前姿态
          Eigen::Vector3d ypr = tools::eulers(buff_solver.R_gimbal2world(), 2, 1, 0);
          tools::draw_text(
            img_debug,
            fmt::format("Gimbal YPR: yaw={:.2f}, pitch={:.2f}, roll={:.2f}", 
                        ypr[0] * 57.3, ypr[1] * 57.3, ypr[2] * 57.3),
            {10, 210}, {255, 255, 255});

          if (power_runes.has_value()) {
            const auto & rune = power_runes.value();

            // --- 绘制识别到的 R 中心（白色十字 + 小圆点）---
            cv::drawMarker(img_debug, rune.r_center, {255, 255, 255}, cv::MARKER_CROSS, 20, 2);
            cv::circle(img_debug, rune.r_center, 3, {255, 255, 255}, -1);
            cv::putText(img_debug, "R_detect", rune.r_center + cv::Point2f(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);

            // 绘制扇叶
            for (const auto & blade : rune.fanblades) {
              cv::Scalar color = {0, 255, 0};  // 默认绿色
              if (blade.type == auto_buff::FanBlade_type::_target) {
                color = {0, 0, 255};  // 目标红色
              } else if (blade.type == auto_buff::FanBlade_type::_unlight) {
                color = {255, 0, 0};  // 未点亮蓝色
              }

              tools::draw_points(img_debug, blade.points, color);
              
              // 绘制关键点索引，帮助排查 PnP 问题
              for (size_t i = 0; i < blade.points.size(); ++i) {
                  cv::putText(img_debug, std::to_string(i), blade.points[i], cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2);
              }
            }

            // 显示命令
            tools::draw_text(
              img_debug,
              fmt::format(
                "Cmd: yaw={:.2f}, pitch={:.2f}, shoot={}", buff_command.yaw * 57.3,
                buff_command.pitch * 57.3, buff_command.shoot),
              {10, 60}, {154, 50, 205});

            // 在图像上显示 XYZ 坐标
            tools::draw_text(
              img_debug, 
              fmt::format("R XYZ: {:.2f}, {:.2f}, {:.2f}", 
                          rune.xyz_in_world.x(), rune.xyz_in_world.y(), rune.xyz_in_world.z()),
              {10, 150}, {0, 255, 255});
            
            tools::draw_text(
              img_debug, 
              fmt::format("Blade XYZ: {:.2f}, {:.2f}, {:.2f}", 
                          rune.blade_xyz_in_world.x(), rune.blade_xyz_in_world.y(), rune.blade_xyz_in_world.z()),
              {10, 180}, {0, 255, 255});

            // 显示 FIRE
            if (buff_command.shoot) {
                tools::draw_text(img_debug, "FIRE", {img_debug.cols - 200, 100}, {0, 0, 255}, 3.0);
            }

          } else {
            tools::draw_text(img_debug, "No Buff Detected", {10, 150}, {0, 0, 255});
          }

          // --- 预测位置可视化（即使掉帧也显示预测，利用EKF的平滑能力）---
          // 1. 绘制当前状态 (当前帧时刻的 EKF 状态) - 绿色
          if (mode.load() == io::Mode::small_buff && !buff_small_target.is_unsolve()) {
              auto Rxyz_in_world_now = buff_small_target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
              auto image_points = buff_solver.reproject_buff(Rxyz_in_world_now, buff_small_target.ekf_x()[4], buff_small_target.ekf_x()[5]);
              if (image_points.size() >= 4) {
                  for (int i = 0; i < 4; i++) {
                      cv::line(img_debug, image_points[i], image_points[(i + 1) % 4], {0, 255, 0}, 2); // 绿色框代表当前状态
                  }
              }
          }

          // 2. 绘制预测状态 (未来弹丸击中时的预测状态) - 青色 (Cyan) {255, 255, 0}
          if (mode.load() == io::Mode::small_buff && pred_small_target.has_value()) {
              auto & target_copy = pred_small_target.value();
              if (target_copy.ekf_x().size() >= 6) {
                  auto Rxyz_in_world_pre = target_copy.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
                  auto image_points = buff_solver.reproject_buff(Rxyz_in_world_pre, target_copy.ekf_x()[4], target_copy.ekf_x()[5]);
                  
                  // 绘制预测装甲板（4个角）- 青色线框
                  if (image_points.size() >= 4) {
                      for (int i = 0; i < 4; i++) {
                          cv::line(img_debug, image_points[i], image_points[(i + 1) % 4], {255, 255, 0}, 2);
                      }
                  }
              }
          } else if (mode.load() == io::Mode::big_buff && pred_big_target.has_value()) {
              auto & target_copy = pred_big_target.value();
              if (target_copy.ekf_x().size() >= 6) {
                  auto Rxyz_in_world_pre = target_copy.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
                  auto image_points = buff_solver.reproject_buff(Rxyz_in_world_pre, target_copy.ekf_x()[4], target_copy.ekf_x()[5]);
                  
                  // 绘制预测装甲板（4个角）- 青色线框
                  if (image_points.size() >= 4) {
                      for (int i = 0; i < 4; i++) {
                          cv::line(img_debug, image_points[i], image_points[(i + 1) % 4], {255, 255, 0}, 2);
                      }
                  }
              }
          }

          // --- 绘制 Buff 轨迹圆 (Visualize Buff Circle Trajectory) ---
          auto draw_circle = [&](const Eigen::VectorXd& x, cv::Scalar color, int thickness) {
              if (x.size() < 6) return;
              std::vector<cv::Point3f> pts_3d;
              // R_yaw=x[0], R_pitch=x[2], R_dis=x[3], yaw=x[4], phase=x[5]
              double R_yaw = x[0];
              double R_pitch = x[2];
              double R_dis = x[3];
              double yaw_constrained = x[4];

              // Calculate Center (World)
              Eigen::Vector3d center(
                  R_dis * std::cos(R_pitch) * std::cos(R_yaw),
                  R_dis * std::cos(R_pitch) * std::sin(R_yaw),
                  R_dis * std::sin(R_pitch));

              // Draw Center Point (圆心)
              auto center_px = buff_solver.world2pixel({cv::Point3f(center.x(), center.y(), center.z())});
              if (!center_px.empty()) {
                  cv::circle(img_debug, center_px[0], thickness + 1, color, -1);
                  std::string label = (color[2] > 200) ? "R_p" : "R_c";
                  cv::putText(img_debug, label, center_px[0] + cv::Point2f(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
              }

              // Draw Circle Points (轨迹圆)
              for (int i = 0; i <= 360; i += 5) { // 步长5度，更平滑
                  double phase = i * CV_PI / 180.0;
                  // Rotation logic matches Target::point_buff2world
                  // assuming roll (x[5]) is the phase angle
                  Eigen::Matrix3d R_buff2world = tools::rotation_matrix(Eigen::Vector3d(yaw_constrained, 0.0, phase));
                  
                  // Blade tip at 0.8m radius (Matches OBJECT_POINTS)
                  Eigen::Vector3d p_buff(0, 0, 0.8);
                  
                  Eigen::Vector3d p_world = R_buff2world * p_buff + center;
                  pts_3d.emplace_back(p_world.x(), p_world.y(), p_world.z());
              }
              
              auto pts_2d = buff_solver.world2pixel(pts_3d);
              if (pts_2d.size() > 2) {
                  std::vector<std::vector<cv::Point>> contours;
                  std::vector<cv::Point> contour;
                  for(auto& p : pts_2d) contour.push_back(p);
                  contours.push_back(contour);
                  cv::drawContours(img_debug, contours, 0, color, thickness);
              }
          };

          // Draw Current Buff Circle (Green)
          if (mode.load() == io::Mode::small_buff && !buff_small_target.is_unsolve()) {
              draw_circle(buff_small_target.ekf_x(), {0, 255, 0}, 2);
          } else if (mode.load() == io::Mode::big_buff && !buff_big_target.is_unsolve()) {
              draw_circle(buff_big_target.ekf_x(), {0, 255, 0}, 2);
          }

          // Draw Predicted Buff Circle (Red) - Use Red to make it very distinct from Green
          // If the circles overlap perfectly, you will see a mixed color or Red on top of Green
          if (mode.load() == io::Mode::small_buff && pred_small_target.has_value()) {
              draw_circle(pred_small_target->ekf_x(), {0, 0, 255}, 2);
          } else if (mode.load() == io::Mode::big_buff && pred_big_target.has_value()) {
              draw_circle(pred_big_target->ekf_x(), {0, 0, 255}, 2);
          }
          // -----------------------------------------------------------

          // 绘制预测打击物理位置（真实目标预测位置，实心小黄点）
          if (pred_xyz.norm() > 0.1) {
              std::vector<cv::Point3f> pred_points_3d = {cv::Point3f(pred_xyz.x(), pred_xyz.y(), pred_xyz.z())};
              auto pred_image_points = buff_solver.world2pixel(pred_points_3d);
              if (!pred_image_points.empty()) {
                  cv::circle(img_debug, pred_image_points[0], 5, {0, 255, 255}, -1); // 实心黄色点代表最终物理击打点
              }
          }

          // cv::resize(img_debug, img_debug, {}, 0.5, 0.5);
          // cv::imshow("Buff Debug", img_debug);
          // cv::waitKey(1);
        }
      }
      
      // 控制循环频率，保持较高的下发频率(~500Hz)以实现预测插值平滑
      std::this_thread::sleep_for(std::chrono::milliseconds(2));

    } else
      continue;
  }

  detect_thread.join();
  buff_result_thread.join();

  return 0;
}
