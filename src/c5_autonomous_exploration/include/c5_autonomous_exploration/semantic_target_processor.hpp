#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <string>
#include <vector>

namespace c5_autonomous_exploration {

struct RawImage { int width; int height; int channels; double stamp; std::vector<unsigned char> data; };
struct ImageTensor { int width; int height; std::vector<float> values; };
struct Detection2d { std::string label; double confidence; double left; double top; double right; double bottom; };
struct CameraModel { double fx; double fy; double cx; double cy; };
struct ProjectedTarget { bool valid; std::string label; double confidence; Vec3 position; double uncertainty; };

struct TargetProcessorConfig {
  int input_width;
  int input_height;
  double minimum_confidence;
  TargetProcessorConfig();
};

class SemanticTargetProcessor {
 public:
  explicit SemanticTargetProcessor(const TargetProcessorConfig& config = TargetProcessorConfig());
  bool loadModel(const std::string& path);
  bool modelLoaded() const;
  ImageTensor preprocess(const RawImage& image) const;
  ProjectedTarget project(const Detection2d& detection, const CameraModel& camera,
                          const Vec3& camera_position, const std::vector<Vec3>& world_points) const;

 private:
  TargetProcessorConfig config_;
  std::string model_path_;
  bool model_loaded_;
};

}
