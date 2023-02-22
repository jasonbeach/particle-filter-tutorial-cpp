#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.kalman.hpp"

#include <numeric>
#include <random>

KalmanParticle::KalmanParticle(double weight_, const Eigen::Vector3d& state_,
                               Eigen::Matrix3d covariance_) :
    SimpleParticle {weight_, state_}, covariance {covariance_} {}

KalmanParticle::KalmanParticle(double weight_, const Eigen::Vector3d& state_) :
    SimpleParticle {weight_, state_}, covariance {Eigen::Matrix3d::Identity()} {}

ParticleFilterKalman::ParticleFilterKalman(
  double number_of_particles, const LimitsParameters& limits,
  const ProcessNoiseParameters& process_noise_params,
  const MeasurementNoiseParameters& measurement_noise_params) :
    particles_ {static_cast<size_t>(number_of_particles)},
    limits_parameters_ {limits},
    process_noise_parameters_ {process_noise_params},
    measurement_noise_parameters_ {measurement_noise_params},
    Q_ {process_noise_params.std_forward, process_noise_params.std_forward,
        process_noise_params.std_angular},
    R_ {measurement_noise_params.std_range, measurement_noise_params.std_angle},
    num_particles_(number_of_particles) {}

void ParticleFilterKalman::initialize_particles_uniform() {
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

void ParticleFilterKalman::multinomial_resampling() {
  particles_ = resample_multinomial(particles_, particles_.size());
}

Eigen::Vector3d ParticleFilterKalman::validate_state(const Eigen::Vector3d& state) {
  Eigen::Vector3d validated_state = state;

  // Make sure state does not exceed allowed limits (cyclic world)
  while (validated_state.x() < limits_parameters_.x_min) {
    validated_state.x() += (limits_parameters_.x_max - limits_parameters_.x_min);
  }
  while (validated_state.x() > limits_parameters_.x_max) {
    validated_state.x() -= (limits_parameters_.x_max - limits_parameters_.x_min);
  }
  while (validated_state.y() < limits_parameters_.y_min) {
    validated_state.y() += (limits_parameters_.y_max - limits_parameters_.y_min);
  }
  while (validated_state.y() > limits_parameters_.y_max) {
    validated_state.y() -= (limits_parameters_.y_max - limits_parameters_.y_min);
  }

  // Angle must be [-pi, pi]
  while (validated_state.z() > M_PI) {
    validated_state.z() -= 2.0 * M_PI;
  }
  while (validated_state.z() < -M_PI) {
    validated_state.z() += 2.0 * M_PI;
  }

  return validated_state;
}

double ParticleFilterKalman::compute_likelihood(const Eigen::Vector3d& sample,
                                                const MeasurementList& measurements,
                                                const LandmarkList& landmarks) {
  // Initialize measurement likelihood
  auto likelihood_sample = 1.0;

  // Loop over all landmarks for current particle
  for (auto [i, lm] : enumerate(landmarks)) {
    //  Compute expected measurement assuming the current particle state
    const auto dx = sample.x() - lm.x();
    const auto dy = sample.y() - lm.y();
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

void ParticleFilterKalman::update(double robot_forward_motion, double robot_angular_motion,
                                  const MeasurementList& measurements,
                                  const LandmarkList& landmarks) {
  KalmanParticleList new_particles;
  new_particles.reserve(particles_.size());

  double sum_weights = 0.0;

  for (const auto& par : particles_) {
    KalmanParticle new_particle;
    new_particle.state = par.state;
    new_particle.covariance = par.covariance;
    Eigen::Vector3d& propagated_state = new_particle.state;
    Eigen::Matrix3d& cov_ekf = new_particle.covariance;
    propagated_state.z() += robot_angular_motion;
    propagated_state.x() += robot_forward_motion * cos(propagated_state.z());
    propagated_state.y() += robot_angular_motion * sin(propagated_state.z());
    propagated_state = validate_state(propagated_state);

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -robot_forward_motion * sin(propagated_state.z());
    F(1, 2) = robot_forward_motion * cos(propagated_state.z());

    cov_ekf = (F * cov_ekf * F.transpose()) + Q_.toDenseMatrix();
    Eigen::Vector3d updated_state_ekf = propagated_state;
    for (const auto& [idx, landmark] : enumerate(landmarks)) {
      const auto dx = updated_state_ekf.x() - landmark.x();
      const auto dy = updated_state_ekf.y() - landmark.y();
      const auto z1_exp = sqrt(dx * dx + dy * dy);
      const auto z2_exp = atan2(dy, dx);

      // Compute Jacobian (dh/dx) around propagated state
      const double H11 = dx / z1_exp;
      const double H12 = dy / z1_exp;
      const double H21 = 1.0 / (1.0 + (dy / dx) * (dy / dx)) * -dy / (dx * dx);
      const double H22 = 1.0 / (1.0 + (dy / dx) * (dy / dx)) * 1.0 / dx;
      Eigen::Matrix<double, 2, 3> H =
        (Eigen::Matrix<double, 2, 3>() << H11, H12, 0.0, H21, H22, 0.0).finished();

      Eigen::Vector2d y_tilde {measurements.at(idx).x() - z1_exp,
                               measurements.at(idx).y() - z2_exp};
      Eigen::Matrix2d S = H * cov_ekf * H.transpose() + R_.toDenseMatrix();
      Eigen::Matrix<double, 3, 2> K = cov_ekf * H.transpose() * S.inverse();
      Eigen::Vector3d delta_state = K * y_tilde;
      updated_state_ekf.x() += delta_state.x();
      updated_state_ekf.y() += delta_state.y();
      updated_state_ekf.z() += delta_state.z();
      updated_state_ekf = validate_state(updated_state_ekf);
      cov_ekf = (Eigen::Matrix3d::Identity() - K * H) * cov_ekf;
    }

    const Eigen::Vector3d updated_state_pf = Mvn {updated_state_ekf, cov_ekf}.sample();
    validate_state(updated_state_pf);
    const auto likelihood = compute_likelihood(updated_state_pf, measurements, landmarks);
    const auto prior =
      Mvn {Eigen::Vector3d::Zero(), Q_.toDenseMatrix()}.pdf(updated_state_pf - propagated_state);
    const auto importance_density =
      Mvn {Eigen::Vector3d::Zero(), cov_ekf}.pdf(updated_state_pf - updated_state_ekf);
    const double weight = likelihood * prior / importance_density;
    sum_weights += weight;
    new_particles.emplace_back(weight, updated_state_pf, cov_ekf);
  }
  if (sum_weights < 1e-10) {
    fmt::print("Warning: sum particles weights very low\n");
  }
  new_particles.normalize_weights();

  particles_ = new_particles;
  multinomial_resampling();
}

const KalmanParticleList& ParticleFilterKalman::particles() const { return particles_; }
