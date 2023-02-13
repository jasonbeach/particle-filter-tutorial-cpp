#pragma once

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"
#include "particle_filter_tutorial_cpp/resampling/resampler.hpp"

class ParticleFilterSIR : public ParticleFilter {
 public:
  ParticleFilterSIR() = default;
  explicit ParticleFilterSIR(
    double number_of_particles, const ParticleFilter::LimitsParameters& limits,
    const ParticleFilter::ProcessNoiseParameters& process_noise_params,
    const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
    ResamplingAlgorithms resampling_algorithm);

  virtual bool needs_resampling() const;

  void update(double robot_forward_motion, double robot_angular_motion,
              const MeasurementList& measurements, const LandmarkList& landmarks) override;

 private:
  ResamplingAlgorithms resampler_;
};