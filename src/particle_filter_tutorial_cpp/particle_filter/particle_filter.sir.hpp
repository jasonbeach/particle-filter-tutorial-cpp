#pragma once

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"
#include "particle_filter_tutorial_cpp/resampling/resampler.hpp"

using ResampleFunction = std::function<bool()>;

class ParticleFilterSIR : public ParticleFilter<SimpleParticle> {
 public:
  ParticleFilterSIR() = default;
  explicit ParticleFilterSIR(
    double number_of_particles, const ParticleFilter::LimitsParameters& limits,
    const ParticleFilter::ProcessNoiseParameters& process_noise_params,
    const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
    ResamplingAlgorithms resampling_algorithm);

  bool needs_resampling() const;

  void update(double robot_forward_motion, double robot_angular_motion,
              const MeasurementList& measurements, const LandmarkList& landmarks) override;
  void set_needs_resampling_function(const ResampleFunction& needs_resampling);

 private:
  ResamplingAlgorithms resampler_;
  ResampleFunction needs_resampling_;
};