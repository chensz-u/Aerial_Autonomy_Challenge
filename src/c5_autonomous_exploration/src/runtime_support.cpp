#include "c5_autonomous_exploration/runtime_support.hpp"

#include <algorithm>
#include <fstream>

namespace c5_autonomous_exploration {

void FlightRecorder::append(const FlightRecord& record) {
  if (!records_.empty() && record.stamp < records_.back().stamp) return;
  records_.push_back(record);
}

bool FlightRecorder::save(const std::string& path) const {
  std::ofstream stream(path.c_str());
  if (!stream.good()) return false;
  for (const FlightRecord& record : records_) {
    stream << record.stamp << ' ' << record.position.x << ' ' << record.position.y << ' ' << record.position.z << ' '
           << record.velocity.x << ' ' << record.velocity.y << ' ' << record.velocity.z << ' ' << record.battery << ' '
           << record.covariance_trace << ' ' << record.mission_state << '\n';
  }
  return stream.good();
}

bool FlightRecorder::load(const std::string& path) {
  std::ifstream stream(path.c_str());
  if (!stream.good()) return false;
  records_.clear();
  FlightRecord record;
  while (stream >> record.stamp >> record.position.x >> record.position.y >> record.position.z >> record.velocity.x >>
         record.velocity.y >> record.velocity.z >> record.battery >> record.covariance_trace >> record.mission_state) records_.push_back(record);
  cursor_ = 0;
  return !records_.empty();
}

bool FlightRecorder::seek(double stamp) {
  const std::vector<FlightRecord>::iterator found = std::lower_bound(records_.begin(), records_.end(), stamp,
      [](const FlightRecord& record, double value) { return record.stamp < value; });
  if (found == records_.end()) return false;
  cursor_ = static_cast<std::size_t>(found - records_.begin());
  return true;
}

bool FlightRecorder::next(FlightRecord* record) {
  if (record == 0 || cursor_ >= records_.size()) return false;
  *record = records_[cursor_++];
  return true;
}

std::size_t FlightRecorder::size() const { return records_.size(); }
void ParameterValidator::requirePositive(const std::string& name, double value) { if (!(value > 0.0)) failures_.push_back(name); }
void ParameterValidator::requireRange(const std::string& name, double value, double minimum, double maximum) { if (value < minimum || value > maximum) failures_.push_back(name); }
bool ParameterValidator::valid() const { return failures_.empty(); }
std::vector<std::string> ParameterValidator::failures() const { return failures_; }

}
