#include "buff_target.hpp"

namespace auto_buff
{
///voter

Voter::Voter() : clockwise_(0) {}

void Voter::vote(const double angle_last, const double angle_now)
{
  if (std::abs(clockwise_) > 50) return;
  
  // 使用 limit_rad 处理角度差，防止在跨越 PI/-PI 时或者跨扇叶时投票错误
  double diff = tools::limit_rad(angle_now - angle_last);
  
  if (diff < 0)
    clockwise_--;
  else if (diff > 0)
    clockwise_++;
}

int Voter::clockwise() { return clockwise_ > 0 ? 1 : -1; }

/// Target

Target::Target() : first_in_(true), unsolvable_(true) {};

Eigen::Vector3d Target::point_buff2world(const Eigen::Vector3d & point_in_buff) const
{
  if (unsolvable_) return Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix3d R_buff2world =
    tools::rotation_matrix(Eigen::Vector3d(ekf_.x[4], 0.0, ekf_.x[5]));  // pitch = 0

  auto R_yaw = ekf_.x[0];
  auto R_pitch = ekf_.x[2];
  auto R_dis = ekf_.x[3];
  Eigen::Vector3d point_in_world =
    R_buff2world * point_in_buff + Eigen::Vector3d(
                                     R_dis * std::cos(R_pitch) * std::cos(R_yaw),
                                     R_dis * std::cos(R_pitch) * std::sin(R_yaw),
                                     R_dis * std::sin(R_pitch));
  return point_in_world;
}

bool Target::is_unsolve() const { return unsolvable_; }

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

/// SmallTarget

SmallTarget::SmallTarget() : Target() {}

void SmallTarget::get_target(
  const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
{
  static std::chrono::steady_clock::time_point start_timestamp = timestamp;
  auto time_gap = tools::delta_time(timestamp, start_timestamp);

  // 如果没有识别，退出函数
  static int lost_cn = 0;
  if (!p.has_value()) {
    lost_cn++;
    if (lost_cn > 40) {
      unsolvable_ = true;
      // tools::logger()->debug("[Target] 丢失buff");
      lost_cn = 0;
      first_in_ = true;
    } else if (!first_in_) {
      // 短时间丢失，进行纯预测
      unsolvable_ = false;
      predict(time_gap - lasttime_);
      lasttime_ = time_gap;
    } else {
      unsolvable_ = true;
    }
    return;
  }

  // init
  if (first_in_) {
    unsolvable_ = true;
    init(time_gap, p.value());
    first_in_ = false;
  }

  // kalman update
  lost_cn = 0;
  unsolvable_ = false;
  update(time_gap, p.value());

  // 处理发散
  // 由于强制设置了速度，这里主要检查是否有严重异常，或者直接注释掉速度检查
  /*
  if (std::abs(ekf_.x[6]) > SMALL_W + CV_PI / 18 || std::abs(ekf_.x[6]) < SMALL_W - CV_PI / 18) {
    unsolvable_ = true;
    tools::logger()->debug("[Target] 小符角度发散spd: {:.2f}", ekf_.x[6] * 180 / CV_PI);
    first_in_ = true;
    return;
  }
  */
}

void SmallTarget::predict(double dt)
{
  // 预测下一个状态
  // clang-format off
  // 修改：移除 R_yaw 的速度预测项 (A_[0,1]=0)，使 R_yaw 在预测阶段保持不变
  // 这实现了 "Center R 只识别不预测" 的要求（除测量更新外）
  A_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // R_yaw
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // v_R_yaw
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // R_pitch
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // R_dis
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // yaw
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,  dt, // roll
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; // spd

  // 过程噪声协方差矩阵                            //// 调整
  auto v1 = 0.01;  // 角加速度方差
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 为 R_yaw, R_pitch, R_dis 添加过程噪声，防止圆心锁定死
  Q_ << a * v1, b * v1,  0.0,  0.0,   0.0,  0.0,  0.0,
        b * v1, c * v1,  0.0,  0.0,   0.0,  0.0,  0.0,
           0.0,    0.0, 1e-5,  0.0,   0.0,  0.0,  0.0, // R_pitch
           0.0,    0.0,  0.0, 1e-4,   0.0,  0.0,  0.0, // R_dis
           0.0,    0.0,  0.0,  0.0,  1e-5,  0.0,  0.0, // yaw
           0.0,    0.0,  0.0,  0.0,   0.0, 1e-4,  0.0, // roll
           0.0,    0.0,  0.0,  0.0,   0.0,  0.0,  0.0;
  // clang-format on 
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = A_ * x;
    x_prior[0] = tools::limit_rad(x_prior[0]);
    x_prior[2] = tools::limit_rad(x_prior[2]);
    x_prior[4] = tools::limit_rad(x_prior[4]);
    x_prior[5] = tools::limit_rad(x_prior[5]);
    return x_prior;
  };
  ekf_.predict(A_, Q_, f);
}

void SmallTarget::init(double nowtime, const PowerRune & p)
{
  // 初始化内部变量
  lasttime_ = nowtime;

  // 初始状态协方差矩阵
  x0_.resize(7);
  P0_.resize(7, 7);
  A_.resize(7, 7);
  Q_.resize(7, 7);
  H_.resize(7, 7);//z x
  R_.resize(7, 7);//z z
  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [angle/row]
  // [spd]   w=CV_PI/6

  // clang-format off
  // 初始状态
  x0_ << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
         p.ypr_in_world[0], p.ypr_in_world[2], 
         -SMALL_W; // 强制顺时针
  // 初始状态协方差矩阵
  P0_ << 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2;
  // 状态转移矩阵
  // A_ 
  // 过程噪声协方差矩阵                            //// 调整
  // Q_ 
  // 测量方程矩阵
  // H_
  // 测量噪声协方差矩阵                            //// 调整
  // R_

  // clang-format on

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[0] = tools::limit_rad(c[0]);
    c[2] = tools::limit_rad(c[2]);
    c[4] = tools::limit_rad(c[4]);
    c[5] = tools::limit_rad(c[5]);
    return c;
  };
  // 创建扩展卡尔曼滤波器对象
  ekf_ = tools::ExtendedKalmanFilter(x0_, P0_, x_add);
}

void SmallTarget::update(double nowtime, const PowerRune & p)
{
  static double last_ypr2 = 0;
  // [R_yaw]     angle0
  // [v_R_yaw]
  // [R_pitch]   angle2
  // [R_dis]
  // [yaw]       angle4
  // [angle/row] angle5
  // [spd]   w=CV_PI/6
  const Eigen::VectorXd & R_ypd = p.ypd_in_world;  // R
  const Eigen::VectorXd & ypr = p.ypr_in_world;
  const Eigen::VectorXd & B_ypd = p.blade_ypd_in_world;  // center of blade

  if (first_in_) {
      last_ypr2 = ypr[2];
  }

  // 处理扇叶跳变 angle/row
  // 使用 limit_rad 处理角度周期性问题，防止在 PI/-PI 边界处判断错误
  if (std::abs(tools::limit_rad(ypr[2] - ekf_.x[5])) > CV_PI / 12) {
    for (int i = -5; i <= 5; i++) {
      double angle_c = ekf_.x[5] + i * 2 * CV_PI / 5;
      // 使用 circular distance 判断是否匹配
      if (std::abs(tools::limit_rad(angle_c - ypr[2])) < CV_PI / 5) {
        ekf_.x[5] = tools::limit_rad(ekf_.x[5] + i * 2 * CV_PI / 5);
        break;
      }
    }
  }

  // vote判断是顺时针还是逆时针旋转
  // 使用测量值的历史差值来投票，更稳定
  voter.vote(last_ypr2, ypr[2]);
  last_ypr2 = ypr[2];
  
  // 强制设置速度大小为 SMALL_W，方向由 voter 决定
  // 恢复自然方向逻辑：预测方向应与投票检测的方向一致
  ekf_.x[6] = voter.clockwise() * SMALL_W;

  // 预测下一个状态
  predict(nowtime - lasttime_);

  // [R_yaw]     angle0
  // [R_pitch]   angle1
  // [R_dis]
  // [angle/row] angle3
  // [B_yaw]     angle4
  // [B_pitch]   angle5
  // [B_dis]

  /// 1.

  // [R_yaw]     angle0
  // [R_pitch]   angle1
  // [R_dis]
  // [yaw]       angle3 (新增，用于约束板子朝向)
  // [angle/row] angle4

  // clang-format off
  Eigen::MatrixXd H1{
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // R_yaw
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}, // R_pitch
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, // R_dis
    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, // yaw (约束 Buff 朝向，解决 bias 问题)
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}  // roll
  };

  Eigen::MatrixXd R1{
    {0.01, 0.0, 0.0, 0.0, 0.0}, // R_yaw
    {0.0, 0.01, 0.0, 0.0, 0.0}, // R_pitch
    {0.0,  0.0, 2.0, 0.0, 0.0}, // R_dis
    {0.0,  0.0, 0.0, 0.001, 0.0}, // yaw (信任 PnP 结果，防止平面抖动)
    {0.0,  0.0, 0.0, 0.0, 0.001}  // roll (极小的噪声以紧跟测量值，消除滞后，确保中心点重合)
  };
  // clang-format on

  // 防止夹角求差出现异常值
  auto z_subtract1 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //5 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]); // yaw
    c[4] = tools::limit_rad(c[4]); // roll
    return c;
  };

  Eigen::VectorXd z1{{R_ypd[0], R_ypd[1], R_ypd[2], ypr[0], ypr[2]}};  // R_ypd, yaw, roll

  // --- 抑制突变逻辑 (已暂时移除，防止因预测方向冲突导致丢失跟踪) ---
  /*
  double diff_angle = std::abs(tools::limit_rad(z1[4] - ekf_.x[5]));
  if (diff_angle > 0.26) { // ~15 degrees
      R1(4, 4) = 100.0;
  }
  */
  // -------------------

  ekf_.update(z1, H1, R1, z_subtract1);

  ///2.

  // [B_yaw]     angle4
  // [B_pitch]   angle5
  // [B_dis]

  // clang-format off
  Eigen::MatrixXd H2 = h_jacobian();  // 3*7

  Eigen::MatrixXd R2{
    {0.01, 0.0, 0.0}, // B_yaw
    {0.0, 0.01, 0.0}, // B_pitch
    {0.0,  0.0, 0.5}  // B_dis
  };
  // clang-format on

  // 定义非线性转换函数h: x -> z
  auto h2 = [&](const Eigen::VectorXd & x) -> Eigen::Vector3d {
    Eigen::VectorXd R_ypd{{x[0], x[2], x[3]}};
    Eigen::VectorXd R_xyz = tools::ypd2xyz(R_ypd);
    Eigen::VectorXd R_xyz_and_yr{{R_ypd[0], R_ypd[1], R_ypd[2], x[4], x[5]}};
    Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.8));
    Eigen::VectorXd B_ypd = tools::xyz2ypd(B_xyz);
    return B_ypd;
  };

  // 防止夹角求差出现异常值
  auto z_subtract2 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //6 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    return c;
  };

  Eigen::VectorXd z2{{B_ypd[0], B_ypd[1], B_ypd[2]}};

  ekf_.update(z2, H2, R2, h2, z_subtract2);

  // 更新lasttime
  lasttime_ = nowtime;
  return;
}

Eigen::MatrixXd SmallTarget::h_jacobian() const
{
  /// Z(3,1) = H3(3,3) * H2(3,5) * H1(5,5) * H0(5,7) * x(7,1)

  // clang-format off
  // 修改：将前三行设为 0，切断 Blade 测量 (z2) 对 Center 状态 (R_yaw, R_pitch, R_dis) 的更新梯度
  // 这实现了 "Center R 和 Blade 分开工作" 的要求，防止 Blade 噪声拉偏 Center
  Eigen::MatrixXd H0{
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // 阻断 R_yaw
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // 阻断 R_pitch
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // 阻断 R_dis
    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}
  };// 5*7

  Eigen::VectorXd R_ypd{{ekf_.x[0], ekf_.x[2], ekf_.x[3]}};
  Eigen::MatrixXd H_ypd2xyz = tools::ypd2xyz_jacobian(R_ypd);  // 3*3
  Eigen::MatrixXd H1{
    {H_ypd2xyz(0, 0), H_ypd2xyz(0, 1), H_ypd2xyz(0, 2), 0.0, 0.0},
    {H_ypd2xyz(1, 0), H_ypd2xyz(1, 1), H_ypd2xyz(1, 2), 0.0, 0.0},
    {H_ypd2xyz(2, 0), H_ypd2xyz(2, 1), H_ypd2xyz(2, 2), 0.0, 0.0},
    {            0.0,             0.0,             0.0, 1.0, 0.0},
    {            0.0,             0.0,             0.0, 0.0, 1.0}
  };// 5*5

  // double pitch = 0;
  double yaw = ekf_.x[4];
  double roll = ekf_.x[5];
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  Eigen::MatrixXd H2{
    {1.0, 0.0, 0.0, 0.8 * cos_yaw * sin_roll, 0.8 * sin_yaw * cos_roll},
    {0.0, 1.0, 0.0, 0.8 * sin_yaw * sin_roll,  0.8 * cos_yaw * cos_roll},
    {0.0, 0.0, 1.0,                       0.0,            0.8 * sin_roll}
  };// 3*5

  Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.8));
  Eigen::MatrixXd H3 = tools::xyz2ypd_jacobian(B_xyz);// 3*3
  // clang-format on

  return H3 * H2 * H1 * H0;  // 3*7

  // auto h2 = [&](const Eigen::VectorXd & x) -> Eigen::Vector3d {
  //   Eigen::VectorXd R_ypd{{x[0], x[2], x[3]}};
  //   Eigen::VectorXd R_xyz = tools::ypd2xyz(R_ypd);
  //   Eigen::VectorXd R_xyz_and_yr{{R_ypd[0], R_ypd[1], R_ypd[2], x[4], x[5]}};
  //   Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  //   Eigen::VectorXd B_ypd = tools::xyz2ypd(B_xyz);
  //   return B_ypd;
  // };
}

/// BigTarget

BigTarget::BigTarget() : Target(), spd_fitter_(100, 0.5, 1.884, 2.000) {}

void BigTarget::get_target(
  const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
{
  static std::chrono::steady_clock::time_point start_timestamp = timestamp;
  auto time_gap = tools::delta_time(timestamp, start_timestamp);

  // 如果没有识别，退出函数
  static int lost_cn = 0;
  if (!p.has_value()) {
    lost_cn++;
    if (lost_cn > 40) {
      unsolvable_ = true;
      // tools::logger()->debug("[Target] 丢失buff");
      lost_cn = 0;
      first_in_ = true;
    } else if (!first_in_) {
      // 短时间丢失，进行纯预测
      unsolvable_ = false;
      predict(time_gap - lasttime_);
      lasttime_ = time_gap;
    } else {
      unsolvable_ = true;
    }
    return;
  }

  // init
  if (first_in_) {
    unsolvable_ = true;
    init(time_gap, p.value());
    first_in_ = false;
  }

  // kalman update
  lost_cn = 0;
  unsolvable_ = false;
  update(time_gap, p.value());

  // 处理发散
  if (
    ekf_.x[7] > 1.045 * 1.5 || ekf_.x[7] < 0.78 / 1.5 || ekf_.x[8] > 2.0 * 1.5 ||
    ekf_.x[8] < 1.884 / 1.5) {
    tools::logger()->debug("[Target] 大符角度发散a: {:.2f}b:{:.2f}", ekf_.x[7], ekf_.x[8]);
    first_in_ = true;
    return;
  }
}

void BigTarget::predict(double dt)
{
  // 预测下一个状态
  double spd = fit_spd_;
  // double spd = ekf_.x[6];
  double a = ekf_.x[7];
  double w = ekf_.x[8];
  double fi = ekf_.x[9];
  double t = lasttime_ + dt;
  // clang-format off
  A_ << 1.0,  dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//R_yaw
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//v_R_yaw
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//R_pitch
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//R_dis
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,//yaw
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, voter.clockwise() * dt , 0.0, 0.0, 0.0,//row
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sin(w * t + fi) - 1, t * a * cos(w * t + fi), a * cos(w * t + fi),//spd
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,//a
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,//w
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;//theta
        
  // 过程噪声协方差矩阵                            //// 调整
  auto v1 = 0.1;  // 角加速度方差 (从 0.9 减小到 0.1 以减少发散)
  auto a1 = dt * dt * dt * dt / 4;
  auto b1 = dt * dt * dt / 2;
  auto c1 = dt * dt;
  // 为 R_yaw, R_pitch, R_dis 添加过程噪声，防止圆心锁定死
  Q_ << a1 * v1, b1 * v1,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        b1 * v1, c1 * v1,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,     0.0, 1e-5,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, // R_pitch
            0.0,     0.0,  0.0, 1e-4,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, // R_dis
            0.0,     0.0,  0.0,  0.0, 1e-5,  0.0,  0.0,  0.0,  0.0,  0.0, // yaw
            0.0,     0.0,  0.0,  0.0,  0.0, 0.09,  0.0,  0.0,  0.0,  0.0, // row
            0.0,     0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0, // spd
            0.0,     0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 1e-3,  0.0,  0.0, // a
            0.0,     0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 1e-3,  0.0, // w
            0.0,     0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0; // fi
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  1.0,  0.0,  0.0,  0.0,// spd  2
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  4.0;

            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0;
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = x;
    x_prior[0] = tools::limit_rad(x_prior[0] + dt * x_prior[1]);
    x_prior[2] = tools::limit_rad(x_prior[2]);
    x_prior[4] = tools::limit_rad(x_prior[4]); // yaw
    x_prior[5] = tools::limit_rad(x_prior[5] + voter.clockwise() * 
    (-a / w * std::cos(w * t + fi) + a / w * std::cos(w * lasttime_ + fi) + (2.09 - a) * dt)); // roll
    x_prior[6] = a * sin(w * t + fi) + 2.09 - a; // spd
    return x_prior;
  };
  // clang-format on
  ekf_.predict(A_, Q_, f);
}

void BigTarget::init(double nowtime, const PowerRune & p)
{
  // 初始化内部变量
  lasttime_ = nowtime;
  unsolvable_ = true;

  // 初始状态协方差矩阵
  x0_.resize(10);
  P0_.resize(10, 10);
  A_.resize(10, 10);
  Q_.resize(10, 10);
  H_.resize(7, 10);
  R_.resize(7, 7);

  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [angle/row]
  // [spd]       角速度 a*sin(wt) + 2.09 - a
  // [a]         0.78-1.045
  // [w]         1.884-2.000
  // [fi]

  // clang-format off
  // 初始状态
  x0_ << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
         p.ypr_in_world[0], p.ypr_in_world[2], 
         1.1775, 0.9125, 1.942, 0.0;//std::atan((spd - 2.09) / 0.9125 + 1
  // 初始状态协方差矩阵
  P0_ << 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 100.0, 0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 400.0;
  // 状态转移矩阵
  // A_
  // 过程噪声协方差矩阵                            //// 调整
  // Q_
  // 测量方程矩阵
  // H_
  // 测量噪声协方差矩阵                            //// 调整
  // R_

  // clang-format on

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[0] = tools::limit_rad(c[0]);
    c[2] = tools::limit_rad(c[2]);
    c[4] = tools::limit_rad(c[4]);
    c[5] = tools::limit_rad(c[5]);
    c[9] = tools::limit_rad(c[9]);
    return c;
  };
  // 创建扩展卡尔曼滤波器对象
  ekf_ = tools::ExtendedKalmanFilter(x0_, P0_, x_add);
}

void BigTarget::update(double nowtime, const PowerRune & p)
{
  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [angle/row] 角度
  // [spd]       角速度 a*sin(wt) + 2.09 - a
  // [a]         0.78-1.045
  // [w]         1.884-2.000
  // [fi]
  const Eigen::VectorXd & R_ypd = p.ypd_in_world;  // R
  const Eigen::VectorXd & ypr = p.ypr_in_world;
  const Eigen::VectorXd & B_ypd = p.blade_ypd_in_world;  // center of blade

  double measured_roll = -ypr[2];

  // 处理扇叶跳变 angle/row
  if (abs(measured_roll - ekf_.x[5]) > CV_PI / 12) {
    for (int i = -5; i <= 5; i++) {
      double angle_c = ekf_.x[5] + i * 2 * CV_PI / 5;
      if (std::fabs(angle_c - measured_roll) < CV_PI / 5) {
        ekf_.x[5] += i * 2 * CV_PI / 5;
        break;
      }
    }
  }

  // vote判断是顺时针还是逆时针旋转
  voter.vote(ekf_.x[5], measured_roll);

  auto anglelast = ekf_.x[5];  ///

  // 预测下一个状态
  predict(nowtime - lasttime_);

  // [R_yaw]     angle0
  // [R_pitch]   angle1
  // [R_dis]
  // [angle/row] angle3
  // [B_yaw]     angle4
  // [B_pitch]   angle5
  // [B_dis]

  /// 1.

  // [R_yaw]     angle0
  // [R_pitch]   angle1
  // [R_dis]
  // [angle/row] angle3

  // clang-format off
  Eigen::MatrixXd H1{
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // R_yaw
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // R_pitch
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // R_dis
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}  // roll
  };

  Eigen::MatrixXd R1{
    {0.01, 0.0, 0.0,  0.0}, // R_yaw
    {0.0, 0.01, 0.0,  0.0}, // R_pitch
    {0.0,  0.0, 0.5,  0.0}, // R_dis
    {0.0,  0.0, 0.0, 0.1}  // roll  (增大噪声以平滑预测)
  };
  // clang-format on

  // 防止夹角求差出现异常值
  auto z_subtract1 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //4 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  Eigen::VectorXd z1{{R_ypd[0], R_ypd[1], R_ypd[2], measured_roll}};  // R_ypd roll

  ekf_.update(z1, H1, R1, z_subtract1);

  ///2.

  // [B_yaw]     angle4
  // [B_pitch]   angle5
  // [B_dis]

  // clang-format off
  Eigen::MatrixXd H2 = h_jacobian();  // 3*10

  Eigen::MatrixXd R2{
    {0.01, 0.0, 0.0}, // B_yaw
    {0.0, 0.01, 0.0}, // B_pitch
    {0.0,  0.0, 0.5}  // B_dis
  };
  // clang-format on

  // 定义非线性转换函数h: x -> z
  auto h2 = [&](const Eigen::VectorXd & x) -> Eigen::Vector3d {
    Eigen::VectorXd R_ypd{{x[0], x[2], x[3]}};
    Eigen::VectorXd R_xyz = tools::ypd2xyz(R_ypd);
    Eigen::VectorXd R_xyz_and_yr{{R_ypd[0], R_ypd[1], R_ypd[2], x[4], x[5]}};
    Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.8));
    Eigen::VectorXd B_ypd = tools::xyz2ypd(B_xyz);
    return B_ypd;
  };

  // 防止夹角求差出现异常值
  auto z_subtract2 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //6 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    return c;
  };

  Eigen::VectorXd z2{{B_ypd[0], B_ypd[1], B_ypd[2]}};

  ekf_.update(z2, H2, R2, h2, z_subtract2);

  // 对ekf速度进行最小二乘拟合 ekf_.x[6] -> fitting_speed -> predict position
  if (ekf_.x[6] < 2.1 && ekf_.x[6] >= 0) spd_fitter_.add_data(nowtime, ekf_.x[6]);
  spd_fitter_.fit();

  fit_spd_ = spd_fitter_.sine_function(
    nowtime, spd_fitter_.best_result_.A, spd_fitter_.best_result_.omega,
    spd_fitter_.best_result_.phi, spd_fitter_.best_result_.C);

  spd = voter.clockwise() * (ekf_.x[5] - anglelast) / (nowtime - lasttime_);  // 仅供调试
  spd = fit_spd_;
  if (std::abs(spd) > 4) spd = 0;

  // 更新lasttime
  lasttime_ = nowtime;
  unsolvable_ = false;
  return;
}

Eigen::MatrixXd BigTarget::h_jacobian() const
{
  /// Z(3,1) = H3(3,3) * H2(3,5) * H1(5,5) * H0(5,10) * x(10,1)

  // clang-format off
  Eigen::MatrixXd H0{
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}
  };// 5*7

  Eigen::VectorXd R_ypd{{ekf_.x[0], ekf_.x[2], ekf_.x[3]}};
  Eigen::MatrixXd H_ypd2xyz = tools::ypd2xyz_jacobian(R_ypd);  // 3*3
  Eigen::MatrixXd H1{
    {H_ypd2xyz(0, 0), H_ypd2xyz(0, 1), H_ypd2xyz(0, 2), 0.0, 0.0},
    {H_ypd2xyz(1, 0), H_ypd2xyz(1, 1), H_ypd2xyz(1, 2), 0.0, 0.0},
    {H_ypd2xyz(2, 0), H_ypd2xyz(2, 1), H_ypd2xyz(2, 2), 0.0, 0.0},
    {            0.0,             0.0,             0.0, 1.0, 0.0},
    {            0.0,             0.0,             0.0, 0.0, 1.0}
  };// 5*5

  // double pitch = 0;
  double yaw = ekf_.x[4];
  double roll = ekf_.x[5];
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  Eigen::MatrixXd H2{
    {1.0, 0.0, 0.0, 0.8 * cos_yaw * sin_roll, 0.8 * sin_yaw * cos_roll},
    {0.0, 1.0, 0.0, 0.8 * sin_yaw * sin_roll,  0.8 * cos_yaw * cos_roll},
    {0.0, 0.0, 1.0,                       0.0,            0.8 * sin_roll}
  };// 3*5

  Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.8));
  Eigen::MatrixXd H3 = tools::xyz2ypd_jacobian(B_xyz);// 3*3
  // clang-format on

  return H3 * H2 * H1 * H0;  // 3*7

  // auto h2 = [&](const Eigen::VectorXd & x) -> Eigen::Vector3d {
  //   Eigen::VectorXd R_ypd{{x[0], x[2], x[3]}};
  //   Eigen::VectorXd R_xyz = tools::ypd2xyz(R_ypd);
  //   Eigen::VectorXd R_xyz_and_yr{{R_ypd[0], R_ypd[1], R_ypd[2], x[4], x[5]}};
  //   Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  //   Eigen::VectorXd B_ypd = tools::xyz2ypd(B_xyz);
  //   return B_ypd;
  // };
}
}  // namespace auto_buff
