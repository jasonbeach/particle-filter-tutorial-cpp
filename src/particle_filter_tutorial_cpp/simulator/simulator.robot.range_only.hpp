#pragma once

#include "particle_filter_tutorial_cpp/simulator/simulator.robot.hpp"

class RobotRange : public Robot {
 public:
  RobotRange() = default;

  RobotRange(const Eigen::Vector3d& initial_state, const Params& params);

  MeasurementList measure(const World& world) const override;
};