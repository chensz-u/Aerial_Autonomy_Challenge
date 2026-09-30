#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"
#include "c5_autonomous_exploration/eskf.hpp"

#include <map>
#include <vector>

namespace c5_autonomous_exploration {

struct FeatureObservation {
  int id;
  double u;
  double v;
  double quality;
};

struct FeatureFrame {
  double stamp;
  std::vector<FeatureObservation> features;
};

struct VioHealth {
  bool initialized;
  bool failed;
  int keyframes;
  int active_tracks;
  double scale;
  double reprojection_rms;
  double confidence;
};

struct VioConfig {
  double keyframe_parallax;
  int minimum_features;
  double maximum_reprojection_rms;
  double track_timeout;
  VioConfig();
};

class VioFrontend {
 public:
  explicit VioFrontend(const VioConfig& config = VioConfig());
  bool ingest(const FeatureFrame& frame);
  void integrateImu(const ImuSample& sample);
  bool initializeScale(const std::vector<Vec3>& visual_positions,
                       const std::vector<Vec3>& metric_positions);
  double evaluateReprojection(const std::vector<Vec3>& observed,
                              const std::vector<Vec3>& projected);
  VioHealth health() const;

 private:
  struct Track {
    double u;
    double v;
    double stamp;
    int observations;
  };
  struct Keyframe {
    double stamp;
    std::map<int, Track> features;
  };

  VioConfig config_;
  std::map<int, Track> tracks_;
  std::vector<Keyframe> keyframes_;
  std::vector<ImuSample> imu_samples_;
  VioHealth health_;
  double parallaxFromLastKeyframe() const;
};

}
