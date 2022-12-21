
#include "particle_filter_tutorial_cpp/simulator/simulator.robot.range_only.hpp"

RobotRange::RobotRange(const Eigen::Vector3d& initial_state, const Params& params) :
    Robot(initial_state, params) {}

MeasurementList RobotRange::measure(const World& world) const {
  MeasurementList measurements;

  std::transform(world.landmarks().begin(), world.landmarks().end(),
                 std::back_inserter(measurements), [this](const Landmark& landmark) {
                   Measurement m {1};  // this robot produces range and bearing measurements
                   const auto dx = current_state_.x() - landmark.x();
                   const auto dy = current_state_.y() - landmark.y();
                   m.x() =
                     get_gaussian_noise_sample(sqrt(dx * dx + dy * dy), params_.std_meas_distance);
                   return m;
                 });
  return measurements;
}