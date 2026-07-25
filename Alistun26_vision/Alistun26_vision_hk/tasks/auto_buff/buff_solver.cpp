#include "buff_solver.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_buff
{
cv::Matx33f Solver::rotation_matrix(double angle) const
{
  return cv::Matx33f(
    1, 0, 0, 0, std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle));
}

void Solver::compute_rotated_points(std::vector<std::vector<cv::Point3f>> & object_points)
{
  const std::vector<cv::Point3f> & base_points = object_points[0];
  for (int i = 1; i < 5; ++i) {
    double angle = i * THETA;
    cv::Matx33f R = rotation_matrix(angle);
    std::vector<cv::Point3f> rotated_points;
    for (const auto & point : base_points) {
      cv::Vec3f vec(point.x, point.y, point.z);
      cv::Vec3f rotated_vec = R * vec;
      rotated_points.emplace_back(rotated_vec[0], rotated_vec[1], rotated_vec[2]);
    }
    object_points[i] = rotated_points;
  }
}

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);

  // compute_rotated_points(OBJECT_POINTS);
  
  std::stringstream ss;
  ss << R_gimbal2imubody_;
  tools::logger()->info("Buff Solver R_gimbal2imubody:\n{}", ss.str());
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
  
  // Debug log to compare with auto_aim
  static int count = 0;
  if (count++ % 60 == 0) {
      Eigen::Vector3d ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);
      tools::logger()->info("Buff Solver set_R_gimbal2world:");
      tools::logger()->info("  q: w={}, x={}, y={}, z={}", q.w(), q.x(), q.y(), q.z());
      tools::logger()->info("  YPR (deg): yaw={:.2f}, pitch={:.2f}, roll={:.2f}", 
                            ypr[0] * 57.3, ypr[1] * 57.3, ypr[2] * 57.3);
  }
}

void Solver::solve(std::optional<PowerRune> & ps) const
{
  if (!ps.has_value()) return;
  PowerRune & p = ps.value();
  // std::vector<cv::Point2f> image_points;
  // std::vector<cv::Point3f> object_points;
  // int i = 0;
  // for (auto & fanblade : p.fanblades) {
  //   if (fanblade.type != _unlight) {
  //     image_points.insert(image_points.end(), fanblade.points.begin(), fanblade.points.end());
  //     image_points.emplace_back(fanblade.center);
  //     object_points.insert(object_points.end(), OBJECT_POINTS[i].begin(), OBJECT_POINTS[i].end());
  //   }
  //   ++i;
  // }
  // image_points.emplace_back(p.r_center);  //r_center
  // object_points.emplace_back(cv::Point3f(0, 0, 0));
  if (p.target().points.size() < 4) {
    tools::logger()->warn("Target points size < 4, skip solvePnP");
    return;
  }

  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point3f> object_points;

  // Add 4 corners
  for (size_t i = 0; i < 4; ++i) {
    image_points.push_back(p.target().points[i]);
    object_points.push_back(OBJECT_POINTS[i]);
  }

  // Add armor center if available
  if (p.target().points.size() >= 5) {
    image_points.push_back(p.target().points[4]);
    object_points.push_back(OBJECT_POINTS[4]);
  }

  // Add R center (移除 R 标中心点参与解算，完全依赖扇叶点位，以确保绿框与扇叶重合)
  // image_points.emplace_back(p.r_center);
  // object_points.push_back(OBJECT_POINTS[6]);

  try {
    cv::solvePnP(
      object_points, image_points, camera_matrix_, distort_coeffs_, rvec_, tvec_, false,
      cv::SOLVEPNP_ITERATIVE);
  } catch (const cv::Exception& e) {
    tools::logger()->error("solvePnP failed: {}", e.what());
    return;
  }

  Eigen::Vector3d t_buff2camera;
  cv::cv2eigen(tvec_, t_buff2camera);
  cv::Mat rmat;
  cv::Rodrigues(rvec_, rmat);
  Eigen::Matrix3d R_buff2camera;
  cv::cv2eigen(rmat, R_buff2camera);

  // CORRECTION: Fix 180 degree phase error (Opposite direction)
  // Rotate Buff Frame by 180 degrees around X-axis
  Eigen::Matrix3d R_correction;
  R_correction << 1, 0, 0,
                  0, -1, 0,
                  0, 0, -1;
  R_buff2camera = R_buff2camera * R_correction;

  // ==== 强制圆心对齐 R 标像素 ====
  // 由于仅用扇叶点解算 PnP，原点(t_buff2camera)会有巨大的杠杆误差导致抖动。
  // 我们使用识别到的 R 标像素(p.r_center)射线，并保持与 PnP 相同的深度 Z。
  // 这样计算出的圆心 3D 坐标在投影回像素时，能完美锁定在 R 标像素上，且深度极稳。
  std::vector<cv::Point2f> r_pts = {p.r_center};
  std::vector<cv::Point2f> r_norm;
  cv::undistortPoints(r_pts, r_norm, camera_matrix_, distort_coeffs_);
  Eigen::Vector3d r_ray(r_norm[0].x, r_norm[0].y, 1.0);
  
  // 使用 PnP 算出的 Z 深度作为交点参数
  double lambda = t_buff2camera.z() / r_ray.z();
  
  // 深度保护：确保 lambda 为正且在合理范围内（1m - 20m）
  lambda = std::max(1.0, std::min(20.0, std::abs(lambda)));
  
  Eigen::Vector3d exact_r_xyz_in_camera = r_ray * lambda;

  // 覆盖原本波动的 t_buff2camera，将圆心强制定为 R 标
  t_buff2camera = exact_r_xyz_in_camera;

  Eigen::Vector3d blade_xyz_in_buff{{0, 0, 800e-3}};

  // buff -> camera
  Eigen::Vector3d xyz_in_camera = t_buff2camera;
  Eigen::Vector3d blade_xyz_in_camera = R_buff2camera * blade_xyz_in_buff + t_buff2camera;

  // camera -> gimbal
  Eigen::Matrix3d R_buff2gimbal = R_camera2gimbal_ * R_buff2camera;
  Eigen::Vector3d xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  Eigen::Vector3d blade_xyz_in_gimbal = R_camera2gimbal_ * blade_xyz_in_camera + t_camera2gimbal_;

  /// gimbal -> world
  Eigen::Matrix3d R_buff2world = R_gimbal2world_ * R_buff2gimbal;

  p.xyz_in_world = R_gimbal2world_ * xyz_in_gimbal;
  p.ypd_in_world = tools::xyz2ypd(p.xyz_in_world);

  p.blade_xyz_in_world = R_gimbal2world_ * blade_xyz_in_gimbal;
  p.blade_ypd_in_world = tools::xyz2ypd(p.blade_xyz_in_world);

  p.ypr_in_world = tools::eulers(R_buff2world, 2, 1, 0);
}

// 调试用
cv::Point2f Solver::point_buff2pixel(cv::Point3f x)
{
  // buff坐标系(单位:m)到像素坐标系
  std::vector<cv::Point3d> world_points;
  std::vector<cv::Point2d> image_points;
  world_points.push_back(x);
  cv::projectPoints(world_points, rvec_, tvec_, camera_matrix_, distort_coeffs_, image_points);
  return image_points.back();
}

// xyz_in_world2xyz_in_pix
std::vector<cv::Point2f> Solver::reproject_buff(
  const Eigen::Vector3d & xyz_in_world, double yaw, double row) const
{
  auto R_buff2world = tools::rotation_matrix(Eigen::Vector3d(yaw, 0.0, row));
  // clang-format on

  // get R_buff2camera t_buff2camera
  const Eigen::Vector3d & t_buff2world = xyz_in_world;
  Eigen::Matrix3d R_buff2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_buff2world;
  Eigen::Vector3d t_buff2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_buff2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_buff2camera_cv;
  cv::eigen2cv(R_buff2camera, R_buff2camera_cv);
  cv::Rodrigues(R_buff2camera_cv, rvec);
  cv::Vec3d tvec(t_buff2camera[0], t_buff2camera[1], t_buff2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(OBJECT_POINTS, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_buff
