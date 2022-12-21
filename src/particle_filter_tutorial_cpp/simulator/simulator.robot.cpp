#include "particle_filter_tutorial_cpp/simulator/simulator.robot.hpp"

#include <random>

Robot::Robot(const Eigen::Vector3d& initial_state, const Params& params) :
    current_state_ {initial_state}, params_ {params} {}

void Robot::move(double desired_distance, double desired_rotation, const World& world) {
  // Compute relative motion (true motion is desired motion with some noise)
  const auto distance_driven = get_gaussian_noise_sample(desired_distance, params_.std_forward);
  const auto angle_rotated = get_gaussian_noise_sample(desired_rotation, params_.std_turn);

  // Update robot pose
  current_state_.z() += angle_rotated;
  current_state_.x() += distance_driven * cos(current_state_.z());
  current_state_.y() += distance_driven * sin(current_state_.z());

  // Cyclic world assumption (i.e. crossing right edge -> enter on left hand side)
  current_state_.x() = std::fmod(current_state_.x(), world.get_size().x());
  current_state_.y() = std::fmod(current_state_.y(), world.get_size().y());

  // Angles in [0, 2*pi]
  current_state_.z() = std::fmod(current_state_.z(), 2.0 * M_PI);
}

MeasurementList Robot::measure(const World& world) const {
  MeasurementList measurements;

  std::transform(world.landmarks().begin(), world.landmarks().end(),
                 std::back_inserter(measurements), [this](const Landmark& landmark) {
                   Measurement m {2};  // this robot produces range and bearing measurements
                   const auto dx = current_state_.x() - landmark.x();
                   const auto dy = current_state_.y() - landmark.y();
                   m.x() =
                     get_gaussian_noise_sample(sqrt(dx * dx + dy * dy), params_.std_meas_distance);
                   m.y() = get_gaussian_noise_sample(atan2(dy, dx), params_.std_meas_angle);
                   return m;
                 });
  return measurements;
}

const Eigen::Vector3d& Robot::state() const { return current_state_; }

double Robot::get_gaussian_noise_sample(double mu, double sigma) {
  static std::random_device rd {};
  static std::mt19937 gen {rd()};

  std::normal_distribution<> dist {mu, sigma};
  return dist(gen);
}