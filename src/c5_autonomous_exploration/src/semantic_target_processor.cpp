#include "c5_autonomous_exploration/semantic_target_processor.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

namespace c5_autonomous_exploration {

TargetProcessorConfig::TargetProcessorConfig() : input_width(640), input_height(384), minimum_confidence(0.35) {}
SemanticTargetProcessor::SemanticTargetProcessor(const TargetProcessorConfig& config) : config_(config), model_loaded_(false) {}

bool SemanticTargetProcessor::loadModel(const std::string& path) {
  std::ifstream stream(path.c_str(), std::ios::binary | std::ios::ate);
  model_loaded_ = stream.good() && stream.tellg() > 0;
  model_path_ = model_loaded_ ? path : "";
  return model_loaded_;
}

bool SemanticTargetProcessor::modelLoaded() const { return model_loaded_; }

ImageTensor SemanticTargetProcessor::preprocess(const RawImage& image) const {
  ImageTensor tensor;
  tensor.width = config_.input_width;
  tensor.height = config_.input_height;
  tensor.values.assign(static_cast<std::size_t>(tensor.width * tensor.height * 3), 0.0f);
  if (image.width <= 0 || image.height <= 0 || image.channels < 3 || image.data.size() < static_cast<std::size_t>(image.width * image.height * image.channels)) return tensor;
  for (int row = 0; row < tensor.height; ++row) {
    const int source_row = std::min(image.height - 1, row * image.height / tensor.height);
    for (int column = 0; column < tensor.width; ++column) {
      const int source_column = std::min(image.width - 1, column * image.width / tensor.width);
      const std::size_t source = static_cast<std::size_t>((source_row * image.width + source_column) * image.channels);
      const std::size_t target = static_cast<std::size_t>((row * tensor.width + column) * 3);
      tensor.values[target] = static_cast<float>(image.data[source]) / 255.0f;
      tensor.values[target + 1] = static_cast<float>(image.data[source + 1]) / 255.0f;
      tensor.values[target + 2] = static_cast<float>(image.data[source + 2]) / 255.0f;
    }
  }
  return tensor;
}

ProjectedTarget SemanticTargetProcessor::project(const Detection2d& detection, const CameraModel& camera,
                                                 const Vec3& camera_position, const std::vector<Vec3>& world_points) const {
  ProjectedTarget target;
  target.valid = false;
  target.label = detection.label;
  target.confidence = detection.confidence;
  target.position = Vec3();
  target.uncertainty = 1.0 - std::max(0.0, std::min(1.0, detection.confidence));
  if (detection.confidence < config_.minimum_confidence || camera.fx <= 0.0 || camera.fy <= 0.0) return target;
  double best_error = std::numeric_limits<double>::infinity();
  for (const Vec3& point : world_points) {
    const double dz = point.z - camera_position.z;
    if (dz <= 1e-6) continue;
    const double u = camera.fx * (point.x - camera_position.x) / dz + camera.cx;
    const double v = camera.fy * (point.y - camera_position.y) / dz + camera.cy;
    if (u < detection.left || u > detection.right || v < detection.top || v > detection.bottom) continue;
    const double center_u = 0.5 * (detection.left + detection.right);
    const double center_v = 0.5 * (detection.top + detection.bottom);
    const double error = std::abs(u - center_u) / camera.fx + std::abs(v - center_v) / camera.fy;
    if (error < best_error) { best_error = error; target.position = point; target.valid = true; }
  }
  if (target.valid) target.uncertainty = std::min(1.0, target.uncertainty + best_error);
  return target;
}

}
