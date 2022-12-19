#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"

#include <numeric>
#include <random>

#include "fmt/color.h"

ParticleFilter::ParticleFilter(double num_particles, const LimitsParameters& limits,
                               const ProcessNoiseParameters& process_noise,
                               const MeasurementNoiseParameters& measurement_noise) :
    limits_parameters_(limits),
    process_noise_parameters_(process_noise),
    measurement_noise_parameters_(measurement_noise),
    particles_(static_cast<size_t>(num_particles)),
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
  static std::random_device rd {};
  static std::mt19937 gen {rd()};
  static std::array<std::normal_distribution<>, 3> dists {
    std::normal_distribution<> {mean_vector[0], std_vector[0]},
    std::normal_distribution<> {mean_vector[1], std_vector[1]},
    std::normal_distribution<> {mean_vector[2], std_vector[2]}};

  const double weight = 1.0 / num_particles_;
  for (auto& particle : particles_) {
    particle.weight = weight;
    particle.state.x() = dists[0](gen);
    particle.state.y() = dists[1](gen);
    particle.state.z() = dists[2](gen);
  }
  fmt::print("{} particles normally initialized\n", particles_.size());
}

Particle ParticleFilter::validate_state(const Particle& particle) {
  auto validated_particle = particle;

  // Make sure state does not exceed allowed limits (cyclic world)
  while (validated_particle.state.x() < limits_parameters_.x_min) {
    validated_particle.state.x() += (limits_parameters_.x_max - limits_parameters_.x_min);
  }
  while (validated_particle.state.x() > limits_parameters_.x_max) {
    validated_particle.state.x() -= (limits_parameters_.x_max - limits_parameters_.x_min);
  }
  while (validated_particle.state.y() < limits_parameters_.y_min) {
    validated_particle.state.y() += (limits_parameters_.y_max - limits_parameters_.y_min);
  }
  while (validated_particle.state.y() > limits_parameters_.y_max) {
    validated_particle.state.y() -= (limits_parameters_.y_max - limits_parameters_.y_min);
  }

  // Angle must be [-pi, pi]
  while (validated_particle.state.z() > M_PI) {
    validated_particle.state.z() -= 2.0 * M_PI;
  }
  while (validated_particle.state.z() < -M_PI) {
    validated_particle.state.z() += 2.0 * M_PI;
  }

  return validated_particle;
}

void ParticleFilter::set_particles(const ParticleList& particles) { particles_ = particles; }

const ParticleList& ParticleFilter::particles() const { return particles_; }

Eigen::Vector3d ParticleFilter::get_average_state() const {
  // compute sum of all weights
  const auto sum_weights =
    std::accumulate(particles_.begin(), particles_.end(), 0.0,
                    [](double sum, const Particle& p) { return sum + p.weight; });

  // Compute weighted average
  return std::accumulate(particles_.begin(), particles_.end(), Eigen::Vector3d {0, 0, 0},
                         [sum_weights](const Eigen::Vector3d& ave, const Particle& p) {
                           return Eigen::Vector3d {ave.x() + p.weight / sum_weights * p.state.x(),
                                                   ave.y() + p.weight / sum_weights * p.state.y(),
                                                   ave.z() + p.weight / sum_weights * p.state.z()};
                         });
}

double ParticleFilter::get_max_weight() const {
  return std::max_element(particles_.begin(), particles_.end(),
                          [](const auto& p1, const auto& p2) { return p1.weight < p2.weight; })
    ->weight;
}

void ParticleFilter::print_particles() const {
  for (auto [i, p] : enumerate(particles_)) {
    fmt::print("({}): {}\n", i, p);
  }
}

ParticleList ParticleFilter::normalize_weights(const ParticleList& particles) {
  const auto num_particles = static_cast<double>(particles.size());
  // compute sum of all weights
  const auto sum_weights =
    std::accumulate(particles.begin(), particles.end(), 0.0,
                    [](double sum, const Particle& p) { return sum + p.weight; });

  ParticleList normalized_particles;
  normalized_particles.reserve(particles.size());

  if (const double new_weight = 1.0 / num_particles; sum_weights < 1e-15) {
    fmt::print(
      "{}: Weight normalization failed: sum of all weights is {} (weights will be reinitialized)\n",
      fmt::styled("Warning", fmt::fg(fmt::color::yellow)), sum_weights);

    std::transform(particles.begin(), particles.end(), std::back_inserter(normalized_particles),
                   [new_weight](const Particle& p) {
                     return Particle {new_weight, p.state};
                   });
    return normalized_particles;
  }

  std::transform(particles.begin(), particles.end(), std::back_inserter(normalized_particles),
                 [sum_weights](const Particle& p) {
                   return Particle {p.weight / sum_weights, p.state};
                 });
  return normalized_particles;
}

Particle ParticleFilter::propagate_sample(const Particle& sample, double forward_motion,
                                          double angular_motion) {
  // no reason to re-initialize these variables every time this function is called, so make them
  // persist as a performance optimization
  static std::random_device rd {};
  static std::mt19937 gen {rd()};

  Particle propagated_sample = sample;

  // 1. rotate by given amount plus additive noise sample
  propagated_sample.state.z() +=
    std::normal_distribution<> {angular_motion, process_noise_parameters_.std_angular}(gen);

  // Compute forward motion by combining deterministic forward motion with additive zero mean
  // Gaussian noise
  const auto forward_displacement =
    std::normal_distribution<> {forward_motion, process_noise_parameters_.std_forward}(gen);

  // 2. move forward
  propagated_sample.state.x() += forward_displacement * cos(propagated_sample.state.z());
  propagated_sample.state.y() += forward_displacement * sin(propagated_sample.state.z());

  // Make sure we stay within cyclic world
  return validate_state(propagated_sample);
}

// this should be in some utility header
static constexpr double gaussian(double x, double mu, double sigma) {
  return exp(-(x - mu) * (x - mu) / (2.0 * sigma * sigma));
}

double ParticleFilter::compute_likelihood(const Particle& sample,
                                          const MeasurementList& measurements,
                                          const LandmarkList& landmarks) {
  // Initialize measurement likelihood
  auto likelihood_sample = 1.0;

  // Loop over all landmarks for current particle
  for (auto [i, lm] : enumerate(landmarks)) {
    //  Compute expected measurement assuming the current particle state
    const auto dx = sample.state.x() - lm.x();
    const auto dy = sample.state.y() - lm.y();
    const auto expected_distance = sqrt(dx * dx + dy * dy);
    const auto expected_bearing = atan2(dy, dx);

    // Map difference true and expected distance measurement to probability
    const auto p_z_given_x_distance =
      gaussian(expected_distance, measurements[i].x(), measurement_noise_parameters_.std_range);

    // Map difference true and expected angle measurement to probability
    const auto p_z_given_x_angle =
      gaussian(expected_bearing, measurements[i].y(), measurement_noise_parameters_.std_angle);

    // Incorporate likelihoods current landmark
    likelihood_sample *= p_z_given_x_distance * p_z_given_x_angle;
  }

  return likelihood_sample;
}