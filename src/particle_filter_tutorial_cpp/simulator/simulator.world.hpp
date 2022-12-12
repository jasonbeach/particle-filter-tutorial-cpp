#pragma once

#include <Eigen/Dense>
#include <vector>

using Landmark = Eigen::Vector2d;
using LandmarkList = std::vector<Landmark>;

class World {
 public:
  World() = default;
  World(const Eigen::Vector2d& size, const LandmarkList& landmarks);

  void update_world_size(const Eigen::Vector2d& world_size);
  const Eigen::Vector2d& get_size() const;

  void update_landmarks(const LandmarkList& landmarks);
  const LandmarkList& landmarks() const;

 private:
  Eigen::Vector2d world_size_;
  LandmarkList landmarks_;
};