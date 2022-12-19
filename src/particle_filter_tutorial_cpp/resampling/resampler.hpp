#pragma once
#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"

/**
 * @brief Particles are sampled with replacement proportional to their weight and in
 * arbitrary order. This leads to a maximum variance on the number of times a particle will be
 * resampled, since any particle will be resampled between 0 and N times.
 *
 *       Computational complexity: O(N log(M)
 *
 * @param samples Samples that must be resampled.
 * @param N Number of samples that must be generated.
 * @return ParticleList Resampled weighted particles.
 */
ParticleList resample_multinomial(const ParticleList& samples, size_t N);

/**
 * @brief Particles should at least be present floor(wi/N) times due to first deterministic loop.
 * First Nt new samples are always the same (when running the function with the same input multiple
 * times).
 *
 *       Computational complexity: O(M) + O(N-Nt),
 *         where Nt is number of samples in first deterministic loop
 *
 * @param samples Samples that must be resampled.
 * @param N Number of samples that must be generated.
 * @return ParticleList Resampled weighted particles.
 */

ParticleList resample_residual(const ParticleList& samples, size_t N);

/**
 * @brief Loop over cumulative sum once hence particles should keep same order (however some
 * disappear, others are replicated).
 *
 *       Computational complexity: O(N)
 *
 * @param samples Samples that must be resampled.
 * @param N Number of samples that must be generated.
 * @return ParticleList Resampled weighted particles.
 */

ParticleList resample_stratified(const ParticleList& samples, size_t N);

/**
 * @brief Loop over cumulative sum once hence particles should keep same order (however some
 * disappear, other are replicated). Variance on number of times a particle will be selected lower
 * than with stratified resampling.
 *
 *       Computational complexity: O(N)
 *
 * @param samples Samples that must be resampled.
 * @param N Number of samples that must be generated.
 * @return ParticleList Resampled weighted particles.
 */

ParticleList resample_systematic(const ParticleList& samples, size_t N);

enum class ResamplingAlgorithms { MULTINOMIAL, RESIDUAL, STRATIFIED, SYSTEMATIC };

ParticleList resample_factory(const ParticleList samples, double N, ResamplingAlgorithms alg);
