#include "yolo11_buff.hpp"

const double ConfidenceThreshold = 0.5f;
const double IouThreshold = 0.4f;
namespace auto_buff
{
YOLO11_BUFF::YOLO11_BUFF(const std::string & config)
{
  auto yaml = YAML::LoadFile(config);
  std::string model_path = yaml["model"].as<std::string>();
  
  // 从 yaml 读取置信度阈值，如果没有则默认 0.70
  if (yaml["min_confidence"]) {
    confidence_threshold_ = yaml["min_confidence"].as<double>();
  } else {
    confidence_threshold_ = 0.70;
  }
  tools::logger()->info("[YOLO11_BUFF] Set confidence_threshold to: {}", confidence_threshold_);

  model = core.read_model(model_path);
  
  // printInputAndOutputsInfo(*model);  // 打印模型信息
  
  // 使用 PrePostProcessor 进行预处理优化
  ov::preprocess::PrePostProcessor ppp(model);
  ov::preprocess::InputInfo& input_info = ppp.input();
  
  // 设置输入张量信息（我们传给它的数据）
  input_info.tensor()
      .set_element_type(ov::element::u8)
      .set_shape({1, 640, 640, 3}) // NHWC
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);
      
  // 设置预处理步骤
  input_info.preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      .scale(255.0f); // 归一化 [0, 255] -> [0, 1]
      
  // 设置模型输入信息（模型期望的数据）
  input_info.model().set_layout("NCHW");
  
  model = ppp.build();

  /// 载入并编译模型
  // 性能优化：设置 OpenVINO 性能提示为 THROUGHPUT，以支持异步推理流水线
  // 强制使用 CPU 并设置线程数，以利用所有核心
  compiled_model = core.compile_model(model, "CPU", 
    ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT),
    ov::inference_num_threads(4),
    ov::streams::num(4),
    ov::hint::enable_cpu_pinning(false)
  );
  /// 创建推理请求
  infer_request = compiled_model.create_infer_request();
  
  // 初始化异步推理请求池
  requests_.reserve(num_requests);
  idle_indices_.reserve(num_requests);
  for (int i = 0; i < num_requests; ++i) {
    requests_.push_back(compiled_model.create_infer_request());
    idle_indices_.push_back(i);
  }
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::get_multicandidateboxes(cv::Mat & image)
{
  const int64 start = cv::getTickCount();  // 设置模型输入

  /// 预处理

  // const float factor = fill_tensor_data_image(input_tensor, image);  // 填充图片到合适的input size

  if (image.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::vector<YOLO11_BUFF::Object> ();
  }

  cv::Mat bgr_img = image;

  const int INPUT_W = 640;
  const int INPUT_H = 640;

  auto x_scale = static_cast<double>(INPUT_H) / bgr_img.rows;
  auto y_scale = static_cast<double>(INPUT_W) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  double factor = 1.0 / scale;  

  // preproces
  auto input = cv::Mat(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  
  // 包装成 Tensor 并设置给 infer_request
  ov::Tensor input_tensor(ov::element::u8, {1, INPUT_H, INPUT_W, 3}, input.data);
  infer_request.set_input_tensor(input_tensor);

  /// 执行推理计算
  infer_request.infer();

  /// 处理推理计算结果
  const ov::Tensor output = infer_request.get_output_tensor();  // 获得推理结果
  const ov::Shape output_shape = output.get_shape();
  const float * output_buffer = output.data<const float>();
  const int out_rows = output_shape[1];  // 获得"output"节点的rows 15
  const int out_cols = output_shape[2];  // 获得"output"节点的cols 8400
  const cv::Mat det_output(
    out_rows, out_cols, CV_32F, (float *)output_buffer);  // output_buff类型转换
  std::vector<cv::Rect> boxes;                            // 目标框
  std::vector<float> confidences;                         // 置信度
  std::vector<std::vector<float>> objects_keypoints;      // 关键点
  // 输出格式是[15,8400], 每列代表一个框(即最多有8400个框), 前面4行分别是[cx, cy, ow, oh], 中间score, 最后5*2关键点(3代表每个关键点的信息, 包括[x, y, visibility],如果是2，则没有visibility)
  // 15 = 4 + 1 + NUM_POINTS * 2      56
  for (int i = 0; i < det_output.cols; ++i) {
    const float score = det_output.at<float>(4, i);
    // 如果置信度满足条件则放进vector
    if (score > ConfidenceThreshold) {
      // 获取目标框
      const float cx = det_output.at<float>(0, i);
      const float cy = det_output.at<float>(1, i);
      const float ow = det_output.at<float>(2, i);
      const float oh = det_output.at<float>(3, i);
      cv::Rect box;
      box.x = static_cast<int>((cx - 0.5 * ow) * factor);
      box.y = static_cast<int>((cy - 0.5 * oh) * factor);
      box.width = static_cast<int>(ow * factor);
      box.height = static_cast<int>(oh * factor);
      boxes.push_back(box);

      // 获取置信度
      confidences.push_back(score);

      // 获取关键点
      std::vector<float> keypoints;
      cv::Mat kpts = det_output.col(i).rowRange(NUM_POINTS, 15);
      for (int j = 0; j < NUM_POINTS; ++j) {
        const float x = kpts.at<float>(j * 2 + 0, 0) * factor;
        const float y = kpts.at<float>(j * 2 + 1, 0) * factor;
        // const float s = kpts.at<float>(j * 3 + 2, 0);
        keypoints.push_back(x);
        keypoints.push_back(y);
        // keypoints.push_back(s);
      }
      objects_keypoints.push_back(keypoints);
    }
  }

  /// NMS,消除具有较低置信度的冗余重叠框,用于处理多个框的情况
  std::vector<int> indexes;
  cv::dnn::NMSBoxes(boxes, confidences, ConfidenceThreshold, IouThreshold, indexes);

  std::vector<Object> object_result;  // 最终得到的object
  for (size_t i = 0; i < indexes.size(); ++i) {
    Object obj;
    const int index = indexes[i];
    obj.rect = boxes[index];
    obj.prob = confidences[index];

    const std::vector<float> & keypoint = objects_keypoints[index];
    for (int i = 0; i < NUM_POINTS; ++i) {
      const float x_coord = keypoint[i * 2];
      const float y_coord = keypoint[i * 2 + 1];
      obj.kpt.push_back(cv::Point2f(x_coord, y_coord));
    }
    
    object_result.push_back(obj);

    /// 绘制关键点和连线
    cv::rectangle(image, obj.rect, cv::Scalar(255, 255, 255), 1, 8);            // 绘制矩形框
    const std::string label = "buff:" + std::to_string(obj.prob).substr(0, 4);  // 绘制标签
    const cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
    const cv::Rect textBox(
      obj.rect.tl().x, obj.rect.tl().y - 15, textSize.width, textSize.height + 5);
    cv::rectangle(image, textBox, cv::Scalar(0, 255, 255), cv::FILLED);
    cv::putText(
      image, label, cv::Point(obj.rect.tl().x, obj.rect.tl().y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(0, 0, 0));
    const int radius = 2;  // 绘制关键点
    const cv::Size & shape = image.size();
    for (int i = 0; i < NUM_POINTS; ++i)
      cv::circle(image, obj.kpt[i], radius, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
  }
  /// 计算FPS
  const float t = (cv::getTickCount() - start) / static_cast<float>(cv::getTickFrequency());
  
  // 性能诊断日志
  static int log_counter = 0;
  // 每帧打印一次，以便快速诊断问题
  tools::logger()->info("YOLO Inference time: {:.2f} ms, Infer FPS: {:.2f}", t * 1000.0, 1.0 / t);

  // cv::putText(
  //   image, cv::format("FPS: %.2f", 1.0 / t), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0,
  //   cv::Scalar(255, 0, 0), 2, 8);

  // #ifdef SAVE
  //         save("save", image);
  // #endif
  return object_result;
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::get_onecandidatebox(cv::Mat & image)
{
  const int64 start = cv::getTickCount();  // 设置模型输入

  if (image.empty()) {
    return std::vector<YOLO11_BUFF::Object> ();
  }

  cv::Mat bgr_img = image;
  const int INPUT_W = 640;
  const int INPUT_H = 640;

  auto x_scale = static_cast<double>(INPUT_H) / bgr_img.rows;
  auto y_scale = static_cast<double>(INPUT_W) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  double factor = 1.0 / scale;

  // preproces
  auto input = cv::Mat(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  
  // 包装成 Tensor 并设置给 infer_request
  ov::Tensor input_tensor(ov::element::u8, {1, INPUT_H, INPUT_W, 3}, input.data);
  infer_request.set_input_tensor(input_tensor);

  /// 执行推理计算

  infer_request.infer();

  /// 处理推理计算结果  output 输出格式是[17,8400], 每列代表一个框(即最多有8400个框), 前面4行分别是[cx, cy, ow, oh], 中间score, 最后6*2关键点

  const ov::Tensor output = infer_request.get_output_tensor();  // 获得推理结果
  const ov::Shape output_shape = output.get_shape();
  const float * output_buffer = output.data<const float>();
  const int out_rows = output_shape[1];  // 获得"output"节点的rows 17
  const int out_cols = output_shape[2];  // 获得"output"节点的cols 8400
  const cv::Mat det_output(
    out_rows, out_cols, CV_32F, (float *)output_buffer);  // output_buff类型转换

  /// 寻找置信度最大的框

  int best_index = -1;
  float max_confidence = 0.0f;
  for (int i = 0; i < det_output.cols; ++i) {
    const float confidence = det_output.at<float>(4, i);
    if (confidence > max_confidence) {
      max_confidence = confidence;
      best_index = i;
    }
  }
  std::vector<Object> object_result;  // 最终得到的object
  if (max_confidence > ConfidenceThreshold) {
    Object obj;
    // 获取目标框
    const float cx = det_output.at<float>(0, best_index);
    const float cy = det_output.at<float>(1, best_index);
    const float ow = det_output.at<float>(2, best_index);
    const float oh = det_output.at<float>(3, best_index);
    obj.rect.x = static_cast<int>((cx - 0.5 * ow) * factor);
    obj.rect.y = static_cast<int>((cy - 0.5 * oh) * factor);
    obj.rect.width = static_cast<int>(ow * factor);
    obj.rect.height = static_cast<int>(oh * factor);
    // 获取置信度
    obj.prob = max_confidence;
    // 获取关键点
    cv::Mat kpts = det_output.col(best_index).rowRange(5, 5 + NUM_POINTS * 2);
    for (int i = 0; i < NUM_POINTS; ++i) {
      const float x = kpts.at<float>(i * 2 + 0, 0) * factor;
      const float y = kpts.at<float>(i * 2 + 1, 0) * factor;
      obj.kpt.push_back(cv::Point2f(x, y));
    }

    object_result.push_back(obj);

    /// 0.3-0.7 save
    if (max_confidence < 0.7) save(std::to_string(start), image);

    /// 绘制关键点和连线
    cv::rectangle(image, obj.rect, cv::Scalar(255, 255, 255), 1, 8);                  // 绘制矩形框
    const std::string label = "buff:" + std::to_string(max_confidence).substr(0, 4);  // 绘制标签
    const cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
    const cv::Rect textBox(
      obj.rect.tl().x, obj.rect.tl().y - 15, textSize.width, textSize.height + 5);
    cv::rectangle(image, textBox, cv::Scalar(0, 255, 255), cv::FILLED);
    cv::putText(
      image, label, cv::Point(obj.rect.tl().x, obj.rect.tl().y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(0, 0, 0));
    const int radius = 2;  // 绘制关键点
    const cv::Size & shape = image.size();
    for (int i = 0; i < NUM_POINTS; ++i) {
      cv::circle(image, obj.kpt[i], radius, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
      cv::putText(
        image, std::to_string(i + 1), obj.kpt[i] + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX,
        0.5, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    }
  }

  /// 计算FPS
  const float t = (cv::getTickCount() - start) / static_cast<float>(cv::getTickFrequency());
  
  // 性能诊断日志
  static int log_counter = 0;
  // 每帧打印一次，以便快速诊断问题
  tools::logger()->info("YOLO Inference time: {:.2f} ms, Infer FPS: {:.2f}", t * 1000.0, 1.0 / t);

  // cv::putText(
  //   image, cv::format("FPS: %.2f", 1.0 / t), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0,
  //   cv::Scalar(255, 0, 0), 2, 8);
  return object_result;
}

void YOLO11_BUFF::convert(
  const cv::Mat & input, cv::Mat & output, const bool normalize, const bool BGR2RGB) const
{
  input.convertTo(output, CV_32F);
  if (normalize) output = output / 255.0;  // 归一化到[0, 1]
  if (BGR2RGB) cv::cvtColor(output, output, cv::COLOR_BGR2RGB);
}

float YOLO11_BUFF::fill_tensor_data_image(ov::Tensor & input_tensor, const cv::Mat & input_image) const
{
  /// letterbox变换: 不改变宽高比(aspect ratio), 将input_image缩放并放置到blob_image左上角
  const ov::Shape tensor_shape = input_tensor.get_shape();
  const size_t num_channels = tensor_shape[1];
  const size_t height = tensor_shape[2];
  const size_t width = tensor_shape[3];
  // 缩放因子
  const float scale = std::min(height / float(input_image.rows), width / float(input_image.cols));
  const cv::Matx23f matrix{
    scale, 0.0, 0.0, 0.0, scale, 0.0,
  };
  cv::Mat blob_image;
  // 下面根据scale范围进行数据转换, 这只是为了提高一点速度(主要是提高了交换通道的速度)
  // 如果不在意这点速度提升的可以固定一种做法(两个if分支随便一个都可以)
  if (scale < 1.0f) {
    // 要缩小, 那么先缩小再交换通道
    cv::warpAffine(input_image, blob_image, matrix, cv::Size(width, height));
    convert(blob_image, blob_image, true, true);
  } else {
    // 要放大, 那么先交换通道再放大
    convert(input_image, blob_image, true, true);
    cv::warpAffine(blob_image, blob_image, matrix, cv::Size(width, height));
  }

  /// 将图像数据填入input_tensor
  float * const input_tensor_data = input_tensor.data<float>();
  // 原有图片数据为 HWC格式，模型输入节点要求的为 CHW 格式
  
  // 优化：使用 cv::split 和 memcpy 替代逐像素拷贝，显著提高性能
  std::vector<cv::Mat> channels(num_channels);
  cv::split(blob_image, channels);
  
  size_t channel_step = width * height;
  for (size_t c = 0; c < num_channels; ++c) {
      // 确保通道数据是连续的
      if (channels[c].isContinuous()) {
          std::memcpy(input_tensor_data + c * channel_step, channels[c].data, channel_step * sizeof(float));
      } else {
          // 如果不连续，回退到行拷贝
          float* dst_ptr = input_tensor_data + c * channel_step;
          for (size_t h = 0; h < height; ++h) {
              const float* src_ptr = channels[c].ptr<float>(h);
              std::memcpy(dst_ptr + h * width, src_ptr, width * sizeof(float));
          }
      }
  }
  
  /* 旧的三重循环实现，效率较低
  for (size_t c = 0; c < num_channels; c++) {
    for (size_t h = 0; h < height; h++) {
      for (size_t w = 0; w < width; w++) {
        input_tensor_data[c * width * height + h * width + w] =
          blob_image.at<cv::Vec<float, 3>>(h, w)[c];
      }
    }
  }
  */
  return 1 / scale;
}

void YOLO11_BUFF::printInputAndOutputsInfo(const ov::Model & network)
{
  std::cout << "model name: " << network.get_friendly_name() << std::endl;

  const std::vector<ov::Output<const ov::Node>> inputs = network.inputs();
  for (const ov::Output<const ov::Node> & input : inputs) {
    std::cout << "    inputs" << std::endl;

    const std::string name = input.get_names().empty() ? "NONE" : input.get_any_name();
    std::cout << "        input name: " << name << std::endl;

    const ov::element::Type type = input.get_element_type();
    std::cout << "        input type: " << type << std::endl;

    const ov::Shape shape = input.get_shape();
    std::cout << "        input shape: " << shape << std::endl;
  }

  const std::vector<ov::Output<const ov::Node>> outputs = network.outputs();
  for (const ov::Output<const ov::Node> & output : outputs) {
    std::cout << "    outputs" << std::endl;

    const std::string name = output.get_names().empty() ? "NONE" : output.get_any_name();
    std::cout << "        output name: " << name << std::endl;

    const ov::element::Type type = output.get_element_type();
    std::cout << "        output type: " << type << std::endl;

    const ov::Shape shape = output.get_shape();
    std::cout << "        output shape: " << shape << std::endl;
  }
}

void YOLO11_BUFF::save(const std::string & programName, const cv::Mat & image)
{
  const std::filesystem::path saveDir = "../result/";
  if (!std::filesystem::exists(saveDir)) {
    std::filesystem::create_directories(saveDir);
  }
  const std::filesystem::path savePath = saveDir / (programName + ".jpg");
  cv::imwrite(savePath.string(), image);
}

void YOLO11_BUFF::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  // 获取空闲请求索引
  int request_index = -1;
  {
    std::lock_guard<std::mutex> lock(idle_mutex_);
    if (!idle_indices_.empty()) {
      request_index = idle_indices_.back();
      idle_indices_.pop_back();
    }
  }

  // 如果没有空闲请求，直接丢弃当前帧（Drop）
  if (request_index == -1) {
    // tools::logger()->warn("[YOLO11_BUFF] No idle request, drop frame!");
    return;
  }

  cv::Mat bgr_img = img;
  const int INPUT_W = 640;
  const int INPUT_H = 640;

  auto x_scale = static_cast<double>(INPUT_H) / bgr_img.rows;
  auto y_scale = static_cast<double>(INPUT_W) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // preproces
  auto input = cv::Mat(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  
  // 使用复用的 infer_request
  auto& request = requests_[request_index];
  
  // 注意：这里必须拷贝数据，因为 input 局部变量会在函数结束时销毁
  // 而异步推理会在后台读取数据。
  // OpenVINO 的 set_input_tensor 如果传入的是外部内存指针，需要保证内存有效性。
  // 更好的做法是获取 request 内部的 tensor 并填充数据。
  
  ov::Tensor input_tensor = request.get_input_tensor();
  // 确保 tensor 形状正确（通常不需要每次设置，除非动态形状）
  // input_tensor.set_shape({1, INPUT_H, INPUT_W, 3}); 
  
  // 填充数据
  std::memcpy(input_tensor.data(), input.data, INPUT_H * INPUT_W * 3);

  // 异步推理
  request.start_async();
  
  // 入队
  busy_queue_.push({img.clone(), t, request_index});
}

std::tuple<std::vector<YOLO11_BUFF::Object>, std::chrono::steady_clock::time_point, cv::Mat> YOLO11_BUFF::pop()
{
  auto task = busy_queue_.pop();
  auto& request = requests_[task.request_index];
  
  // 等待推理完成
  request.wait();

  // 后处理逻辑
  cv::Mat img = task.img;
  const int INPUT_W = 640;
  const int INPUT_H = 640;
  auto x_scale = static_cast<double>(INPUT_H) / img.rows;
  auto y_scale = static_cast<double>(INPUT_W) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  double factor = 1.0 / scale;

  const ov::Tensor output = request.get_output_tensor();
  const ov::Shape output_shape = output.get_shape();
  const float * output_buffer = output.data<const float>();
  const int out_rows = output_shape[1];
  const int out_cols = output_shape[2];
  const cv::Mat det_output(out_rows, out_cols, CV_32F, (float *)output_buffer);

  int best_index = -1;
  float max_confidence = 0.0f;
  for (int i = 0; i < det_output.cols; ++i) {
    const float confidence = det_output.at<float>(4, i);
    if (confidence > max_confidence) {
      max_confidence = confidence;
      best_index = i;
    }
  }
  std::vector<Object> object_result;
  if (max_confidence > confidence_threshold_) {
    Object obj;
    const float cx = det_output.at<float>(0, best_index);
    const float cy = det_output.at<float>(1, best_index);
    const float ow = det_output.at<float>(2, best_index);
    const float oh = det_output.at<float>(3, best_index);
    obj.rect.x = static_cast<int>((cx - 0.5 * ow) * factor);
    obj.rect.y = static_cast<int>((cy - 0.5 * oh) * factor);
    obj.rect.width = static_cast<int>(ow * factor);
    obj.rect.height = static_cast<int>(oh * factor);
    obj.prob = max_confidence;
    cv::Mat kpts = det_output.col(best_index).rowRange(5, 5 + NUM_POINTS * 2);
    for (int i = 0; i < NUM_POINTS; ++i) {
      const float x = kpts.at<float>(i * 2 + 0, 0) * factor;
      const float y = kpts.at<float>(i * 2 + 1, 0) * factor;
      obj.kpt.push_back(cv::Point2f(x, y));
    }
    object_result.push_back(obj);
  }
  
  // 归还请求索引
  {
    std::lock_guard<std::mutex> lock(idle_mutex_);
    idle_indices_.push_back(task.request_index);
  }

  return {object_result, task.t, img};
}

}  // namespace auto_buff
