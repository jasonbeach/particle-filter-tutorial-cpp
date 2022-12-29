#pragma once

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"
#include "particle_filter_tutorial_cpp/resampling/resampler.hpp"

class ParticleFilterAuxiliary : public ParticleFilter {
 public:
  ParticleFilterAuxiliary() = default;
  explicit ParticleFilterAuxiliary(
    double number_of_particles, const ParticleFilter::LimitsParameters& limits,
    const ParticleFilter::ProcessNoiseParameters& process_noise_params,
    const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params);

  /**
   * @brief Process a measurement given the measured robot displacement and adopting the
   * auxiliary sampling importance core (ASIR) scheme explained in [1].
   *
   * [1] M. S. Arulampalam, S. Maskell, N. Gordon and T. Clapp, "A tutorial on particle filters for
   * online nonlinear/non-Gaussian Bayesian tracking," in IEEE Transactions on Signal Processing,
   * vol. 50, no. 2, pp. 174-188, Feb. 2002.
   *
   * @param robot_forward_motion Measured forward robot motion in meters.
   * @param robot_angular_motion Measured angular robot motion in radians.
   * @param measurements Measurements.
   * @param landmarks Landmark positions.
   */
  void update(double robot_forward_motion, double robot_angular_motion,
              const MeasurementList& measurements, const LandmarkList& landmarks) override;
};