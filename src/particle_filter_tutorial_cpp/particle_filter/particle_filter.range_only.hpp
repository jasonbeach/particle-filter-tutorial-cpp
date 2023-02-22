#pragma once
#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"
#include "particle_filter_tutorial_cpp/resampling/resampler.hpp"

class ParticleFilterRangeOnly : public ParticleFilter<SimpleParticle> {
 public:
  ParticleFilterRangeOnly() = default;
  ParticleFilterRangeOnly(double num_particles, const ParticleFilter::LimitsParameters& limits,
                          const ParticleFilter::ProcessNoiseParameters& process_noise,
                          const ParticleFilter::MeasurementNoiseParameters& measurement_noise,
                          ResamplingAlgorithms resampling_algorithm);

  bool needs_resampling() const;

  double compute_likelihood(const SimpleParticle& sample, const MeasurementList& measurements,
                            const LandmarkList& landmarks) override;

  void update(double robot_forward_motion, double robot_angular_motion,
              const MeasurementList& measurements, const LandmarkList& landmarks) override;

 private:
  ResamplingAlgorithms resampler_;
};
