#include "mt_detector.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();

  auto model = core_.read_model(model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})  // TODO
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));

  infer_requests_.reserve(num_requests_);
  idle_indices_.reserve(num_requests_);
  for (int i = 0; i < num_requests_; ++i) {
    infer_requests_.push_back(compiled_model_.create_infer_request());
    idle_indices_.push_back(i);
  }

  tools::logger()->info("[MultiThreadDetector] initialized !");
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  int request_index = -1;
  {
    std::lock_guard<std::mutex> lock(idle_mutex_);
    if (!idle_indices_.empty()) {
      request_index = idle_indices_.back();
      idle_indices_.pop_back();
    }
  }

  if (request_index == -1) {
    return; // Drop frame if no idle request
  }

  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(img, input(roi), {w, h});

  auto& infer_request = infer_requests_[request_index];
  ov::Tensor input_tensor = infer_request.get_input_tensor();
  std::memcpy(input_tensor.data(), input.data, 640 * 640 * 3);

  infer_request.start_async();
  busy_queue_.push({img.clone(), t, request_index});
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto task = busy_queue_.pop();
  auto& infer_request = infer_requests_[task.request_index];
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / task.img.rows;
  auto y_scale = static_cast<double>(640) / task.img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, task.img, 0);  //暂不支持ROI

  {
    std::lock_guard<std::mutex> lock(idle_mutex_);
    idle_indices_.push_back(task.request_index);
  }

  return {std::move(armors), task.t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  auto task = busy_queue_.pop();
  auto& infer_request = infer_requests_[task.request_index];
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / task.img.rows;
  auto y_scale = static_cast<double>(640) / task.img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, task.img, 0);  //暂不支持ROI

  {
    std::lock_guard<std::mutex> lock(idle_mutex_);
    idle_indices_.push_back(task.request_index);
  }

  return {task.img, std::move(armors), task.t};
}

}  // namespace multithread

}  // namespace auto_aim
