#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <map>
#include <string>
#include <vector>

namespace c5_autonomous_exploration {

struct FlightRecord {
  double stamp;
  Vec3 position;
  Vec3 velocity;
  double battery;
  double covariance_trace;
  int mission_state;
};

class FlightRecorder {
 public:
  void append(const FlightRecord& record);
  bool save(const std::string& path) const;
  bool load(const std::string& path);
  bool seek(double stamp);
  bool next(FlightRecord* record);
  std::size_t size() const;

 private:
  std::vector<FlightRecord> records_;
  std::size_t cursor_ = 0;
};

class ParameterValidator {
 public:
  void requirePositive(const std::string& name, double value);
  void requireRange(const std::string& name, double value, double minimum, double maximum);
  bool valid() const;
  std::vector<std::string> failures() const;

 private:
  std::vector<std::string> failures_;
};

}
