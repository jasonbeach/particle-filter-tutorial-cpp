#pragma once
#include <vector>

#include "Eigen/Dense"

class ParticleFilter {
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
  struct Particle {
    // shouldn't hardcode state vector as size 3, but ok for now: x, y, theta
    Eigen::Vector3d state;
    double weight;
  };
  ParticleFilter(double num_particles, const LimitsParameters& limits,
                 const ProcessNoiseParameters& process_noise,
                 const MeasurementNoiseParameters& measurement_noise);

  void initialize_particles_uniform();

 private:
  LimitsParameters limits_parameters_;
  ProcessNoiseParameters process_noise_parameters_;
  MeasurementNoiseParameters measurement_noise_parameters_;
  std::vector<Particle> particles_;
  double num_particles_;
};