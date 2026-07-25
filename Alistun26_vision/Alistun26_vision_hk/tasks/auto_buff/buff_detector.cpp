#include "buff_detector.hpp"

#include "tools/logger.hpp"

namespace auto_buff
{
Buff_Detector::Buff_Detector(const std::string & config) : status_(LOSE), lose_(0), MODE_(config) {}

void Buff_Detector::handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img)
{
  // 彩色图转灰度图
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);  // 彩色图转灰度图
  // cv::imshow("gray", gray_img);  // 调试用

  // 进行二值化           :把高于100变成255，低于100变成0
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, 100, 255, cv::THRESH_BINARY);
  // cv::imshow("binary", binary_img);  // 调试用

  // 膨胀
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  // 使用矩形核
  cv::dilate(binary_img, dilated_img, kernel, cv::Point(-1, -1), 1);
  // cv::imshow("Dilated Image", dilated_img);  // 调试用
}

cv::Point2f Buff_Detector::get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img)
{
  /// error

  if (fanblades.empty()) {
    tools::logger()->debug("[Buff_Detector] 无法计算r_center!");
    return {0, 0};
  }

  /// 直接使用神经网络输出的第6个点（索引5）作为 R标中心
  cv::Point2f r_center_t = {0, 0};
  for (auto & fanblade : fanblades) {
    if (fanblade.points.size() > 5) {
      r_center_t += fanblade.points[5];
    } else {
      // 兼容没有第6个点的情况，尽管通常会有
      r_center_t += fanblade.points[4];
    }
  }
  r_center_t /= float(fanblades.size());

  // 调试用，画出计算出的 R 标中心
  tools::draw_point(bgr_img, r_center_t, {0, 255, 255}, 5);

  return r_center_t;
};

void Buff_Detector::handle_lose()
{
  lose_++;
  if (lose_ >= LOSE_MAX) {
    status_ = LOSE;
    last_powerrune_ = std::nullopt;
  }
  status_ = TEM_LOSE;
}

std::optional<PowerRune> Buff_Detector::detect_24(cv::Mat & bgr_img)
{
  /// onnx 模型检测

  std::vector<YOLO11_BUFF::Object> results = MODE_.get_multicandidateboxes(bgr_img);

  /// 处理未获得的情况

  if (results.empty()) {
    handle_lose();
    return std::nullopt;
  }

  /// results转扇叶FanBlade

  std::vector<FanBlade> fanblades;
  for (auto & result : results) fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

  /// 生成PowerRune
  auto r_center = get_r_center(fanblades, bgr_img);
  PowerRune powerrune(fanblades, r_center, last_powerrune_);

  /// handle error
  if (powerrune.is_unsolve()) {
    handle_lose();
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  return P;
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
  /// onnx 模型检测

  std::vector<YOLO11_BUFF::Object> results = MODE_.get_onecandidatebox(bgr_img);

  /// 处理未获得的情况

  if (results.empty()) {
    handle_lose();
    return std::nullopt;
  }

  /// results转扇叶FanBlade

  std::vector<FanBlade> fanblades;
  auto result = results[0];
  fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

  /// 生成PowerRune
  auto r_center = get_r_center(fanblades, bgr_img);
  PowerRune powerrune(fanblades, r_center, last_powerrune_);

  /// handle error
  if (powerrune.is_unsolve()) {
    handle_lose();
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  return P;
}

void Buff_Detector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  MODE_.push(img, t);
}

std::tuple<std::optional<PowerRune>, std::chrono::steady_clock::time_point, cv::Mat> Buff_Detector::pop()
{
  auto [results, t, img] = MODE_.pop();

  /// 处理未获得的情况

  if (results.empty()) {
    handle_lose();
    return {std::nullopt, t, img};
  }

  /// results转扇叶FanBlade

  std::vector<FanBlade> fanblades;
  auto result = results[0];
  fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

  /// 生成PowerRune
  auto r_center = get_r_center(fanblades, img);
  PowerRune powerrune(fanblades, r_center, last_powerrune_);

  /// handle error
  if (powerrune.is_unsolve()) {
    handle_lose();
    return {std::nullopt, t, img};
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  return {P, t, img};
}

std::optional<PowerRune> Buff_Detector::detect_debug(cv::Mat & bgr_img, cv::Point2f v)
{
  /// onnx 模型检测

  std::vector<YOLO11_BUFF::Object> results = MODE_.get_multicandidateboxes(bgr_img);

  /// 处理未获得的情况

  if (results.empty()) return std::nullopt;

  /// results转扇叶FanBlade

  std::vector<FanBlade> fanblades_t;
  for (auto & result : results)
    fanblades_t.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

  /// 计算r_center,筛选fanblade
  auto r_center = get_r_center(fanblades_t, bgr_img);
  std::vector<FanBlade> fanblades;
  for (auto & fanblade : fanblades_t) {
    if (cv::norm((fanblade.center - r_center) - v) < 10 || results.size() == 1) {
      fanblades.emplace_back(fanblade);
      break;
    }
  }
  if (fanblades.empty()) return std::nullopt;
  PowerRune powerrune(fanblades, r_center, std::nullopt);

  std::optional<PowerRune> P;
  P.emplace(powerrune);
  return P;
}

}  // namespace auto_buff
