#include "c5_autonomous_exploration/dynamic_scene_filter.hpp"

#include <cassert>
#include <vector>

using c5_autonomous_exploration::DynamicSceneFilter;
using c5_autonomous_exploration::Vec3;

int main() {
  DynamicSceneFilter filter;
  filter.update(std::vector<Vec3>{Vec3(0.0, 0.0, 0.0), Vec3(0.1, 0.0, 0.0)}, 1.0);
  assert(filter.tracks().size() == 2);
  assert(filter.staticPoints().size() == 2);
  assert(filter.dynamicPoints().empty());

  filter.update(std::vector<Vec3>{Vec3(0.02, 0.0, 0.0), Vec3(0.12, 0.0, 0.0)}, 1.1);
  assert(filter.tracks().size() == 2);
  assert(filter.staticPoints().size() == 2);
  assert(filter.dynamicPoints().empty());
  return 0;
}
