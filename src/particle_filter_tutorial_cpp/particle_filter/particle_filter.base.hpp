#pragma once
#include <tuple>

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.types.hpp"

/**
 * @brief Notes:
 *        - State is (x, y, heading), where x and y are in meters and heading in radians
 *        - State space assumed limited size in each dimension, world is cyclic (hence leaving at
 *          x_max means entering at x_min)
 *        - Abstract class
 *
 */
template <class ParticleType>
class ParticleFilter {
  using ParticleListType = ParticleList<ParticleType>;

 public:
  struct LimitsParameters {
    double x_min = 0.;
    double x_max = 10.;
    double y_min = 0.;
    double y_max = 10.;
  };
  struct ProcessNoiseParameters {
    double std_forward = 0.;
    double std_angular = 0.;
  };
  struct MeasurementNoiseParameters {
    double std_range = 0.;
    double std_angle = 0.;
  };

  ParticleFilter() = default;
  ParticleFilter(double num_particles, const LimitsParameters& limits,
                 const ProcessNoiseParameters& process_noise,
                 const MeasurementNoiseParameters& measurement_noise) :
      limits_parameters_(limits),
      process_noise_parameters_(process_noise),
      measurement_noise_parameters_(measurement_noise),
      particles_(static_cast<size_t>(num_particles)),
      num_particles_(num_particles) {
    particles_.resize(static_cast<size_t>(num_particles));
    if (num_particles_ < 1) {
      fmt::print("{}: initializing particle filter with number of particles < 1: {}\n",
                 fmt::styled("Warning", fmt::fg(fmt::color::yellow)), num_particles_);
    }
  }

  virtual ~ParticleFilter() = default;

  /**
   * @brief Initialize the particles uniformly over the world assuming a 3D state (x, y, heading).
   * No arguments are required and function always succeeds hence no return value.
   *
   */
  void initialize_particles_uniform() {
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

  /**
   * @brief Initialize particle filter using a Gaussian distribution with dimension three: x, y,
   * heading. Only standard deviations can be provided hence the covariances are all assumed zero.
   * Again the state dimension is implicitly hardcoded here by assuming each vector has a length of
   * 3.
   *
   * @param mean_vector Mean of the Gaussian distribution used for initializing the particle states
   * @param std_vector Standard deviations (one for each dimension)
   */
  void initialize_particles_gaussian(const Eigen::Vector3d& mean_vector,
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

  /**
   * @brief Validate the state. State values outide allowed ranges will be corrected for assuming a
   * 'cyclic world'.
   *
   * @param particle Input particle state.
   * @return Particle Validated particle state.
   */
  ParticleType validate_state(const ParticleType& particle) {
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

  /**
   * @brief Initialize the particle filter using the given set of particles.
   *
   * @param particles Initial particle set.
   */
  void set_particles(const ParticleListType& particles) { particles_ = particles; }

  /**
   * @brief get particles
   *
   * @return const ParticleList&
   */
  const ParticleListType& particles() const { return particles_; }

  /**
   * @brief Propagate an individual sample with a simple motion model that assumes the robot rotates
   * angular_motion rad and then moves forward_motion meters in the direction of its heading. Return
   * the propagated sample (leave input unchanged).
   *
   * @param sample Sample (unweighted particle) that must be propagated
   * @param forward_motion Forward motion in meters
   * @param angular_motion Angular motion in radians
   * @return Particle propagated sample
   */
  ParticleType propagate_sample(const ParticleType& sample, double forward_motion,
                                double angular_motion) {
    // no reason to re-initialize these variables every time this function is called, so make them
    // persist as a performance optimization
    static std::random_device rd {};
    static std::mt19937 gen {rd()};

    ParticleType propagated_sample = sample;

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

  /**
   * @brief Compute likelihood p(z|sample) for a specific measurement given sample state and
   * landmarks.
   *
   * @param sample sample for which likelihood is to be computed
   * @param measurements List with measurements, for each landmark [distance_to_landmark,
   * angle_wrt_landmark], units are meters and radians
   * @param landmarks Positions (absolute) landmarks (in meters)
   * @return double
   */
  virtual double compute_likelihood(const ParticleType& sample, const MeasurementList& measurements,
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

  /**
   * @brief Process a measurement given the measured robot displacement. Abstract method that must
   * be implemented in derived class.
   *
   * @param robot_forward_motion Measured forward robot motion in meters.
   * @param robot_angular_motion Measured angular robot motion in radians.
   * @param measurements Measurements
   * @param landmarks Landmark Positions
   */
  virtual void update(double robot_forward_motion, double robot_angular_motion,
                      const MeasurementList& measurements, const LandmarkList& landmarks) = 0;

 protected:
  LimitsParameters limits_parameters_;
  ProcessNoiseParameters process_noise_parameters_;
  MeasurementNoiseParameters measurement_noise_parameters_;
  ParticleList<ParticleType> particles_;
  double num_particles_;
};
