#include "particle_filter/particle_filter.base.hpp"

#include <random>

#include "fmt/color.h"
#include "fmt/core.h"

ParticleFilter::ParticleFilter(double num_particles, const LimitsParameters& limits,
                               const ProcessNoiseParameters& process_noise,
                               const MeasurementNoiseParameters& measurement_noise) :
    limits_parameters_(limits),
    process_noise_parameters_(process_noise),
    measurement_noise_parameters_(measurement_noise),
    particles_(num_particles),
    num_particles_(num_particles) {
  if (num_particles_ < 1) {
    fmt::print("{}: initializing particle filter with number of particles < 1: {}\n",
               fmt::styled("Warning", fmt::fg(fmt::color::yellow)), num_particles_);
  }
}

void ParticleFilter::initialize_particles_uniform() {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_real_distribution<> angle_dis(0.0, 2.0 * M_PI);
  std::uniform_real_distribution<> limits_dis(0.0, 1.0);
  const double weight = 1.0 / num_particles_;
  for (auto& particle : particles_) {
    particle.weight = weight;
    particle.state.x() = limits_parameters_.x_min +
                         limits_dis(gen) * (limits_parameters_.x_max - limits_parameters_.x_min);
    particle.state.y() = limits_parameters_.y_min +
                         limits_dis(gen) * (limits_parameters_.y_max - limits_parameters_.y_min);
    particle.state.z() = angle_dis(gen);
  }
  fmt::print("{} particles uniformly initialized\n", particles_.size());
}

void ParticleFilter::initialize_particles_gaussian(const Eigen::Vector3d& mean_vector,
                                                   const Eigen::Vector3d& std_vector) {
  std::random_device rd {};
  std::mt19937 gen {rd()};
  std::array<std::normal_distribution<>, 3> dists;
  dists[0] = std::normal_distribution<> {mean_vector[0], std_vector[0]};
  dists[1] = std::normal_distribution<> {mean_vector[1], std_vector[1]};
  dists[2] = std::normal_distribution<> {mean_vector[2], std_vector[2]};

  const double weight = 1.0 / num_particles_;
  for (auto& particle : particles_) {
    particle.weight = weight;
    particle.state.x() = dists[0](gen);
    particle.state.y() = dists[1](gen);
    particle.state.z() = dists[2](gen);
  }
  fmt::print("{} particles normally initialized\n", particles_.size());
}
