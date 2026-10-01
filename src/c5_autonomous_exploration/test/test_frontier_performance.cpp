#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <cassert>
#include <chrono>
#include <iostream>

using c5_autonomous_exploration::AutonomousExplorer;
using c5_autonomous_exploration::CellState;
using c5_autonomous_exploration::ExplorerConfig;
using c5_autonomous_exploration::Vec3;

int main() {
  ExplorerConfig config;
  config.map_resolution = 0.25;
  config.min_clearance = 0.60;
  AutonomousExplorer explorer(config);
  const int voxel_count = 6000;
  for (int index = 0; index < voxel_count; ++index) {
    explorer.setVoxel(Vec3(static_cast<double>(index * 2) * config.map_resolution, 0.0, 0.0), CellState::kFree);
  }

  const auto start = std::chrono::steady_clock::now();
  const auto frontiers = explorer.frontiers();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
  std::cout << "frontiers=" << frontiers.size() << " elapsed_ms=" << elapsed.count() << "\n";
  assert(frontiers.size() == static_cast<std::size_t>(voxel_count));
  assert(elapsed < std::chrono::seconds(5));
  ExplorerConfig clearance_config;
  clearance_config.map_resolution = 1.0;
  clearance_config.min_clearance = 1.5;
  AutonomousExplorer clearance_explorer(clearance_config);
  clearance_explorer.setVoxel(Vec3(0.0, 0.0, 0.0), CellState::kFree);
  clearance_explorer.setVoxel(Vec3(1.0, 0.0, 0.0), CellState::kFree);
  clearance_explorer.setVoxel(Vec3(3.0, 0.0, 0.0), CellState::kFree);
  clearance_explorer.setVoxel(Vec3(1.0, 1.0, 0.0), CellState::kOccupied);
  const auto safe_frontiers = clearance_explorer.frontiers();
  assert(safe_frontiers.size() == 1);
  assert(std::abs(safe_frontiers.front().x - 3.5) < 1e-9);
  return 0;
}
