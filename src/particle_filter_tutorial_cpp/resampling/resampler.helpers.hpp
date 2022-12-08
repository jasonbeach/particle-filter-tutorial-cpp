#pragma once

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.types.hpp"

/**
 * @brief Compute cumulative sum of a list of scalar weights
 *
 * @param samples list with weights
 * @return std::vector<double> list containing cumulative weights, length equal to length input
 */
ParticleList cumulative_sum(const ParticleList& samples);

/**
 * @brief Find the element in a sample list for which element.weight < x <= next_element.weight
 * within cumulativeList[lower:upper] using a naive search method
 *
 * @param cumulative_list List of elements that increase with increasing index.
 * @param x value for which has to be checked
 * @return size_t Index
 */
ParticleList::const_iterator naive_search(const ParticleList& cumulative_list, double x);

/**
 * @brief Find the element in a sample list for which element.weight < x <= next_element.weight
 * within cumulativeList[lower:upper] using a binary search method
 *
 * @param cumulative_list List of elements that increase with increasing index.
 * @param x value for which has to be checked
 * @return size_t Index
 */
ParticleList::const_iterator binary_search(const ParticleList& cumulative_list, double x);

ParticleList::const_iterator draw_sample_by_weight(const ParticleList& cumulative_list);

/**
 * @brief simple rounding function to robustly convert a double to int with a guarentee that
 * floating point precision won't be an issue.  i.e. if we want 15 but the number given
 * is 14.999999999999999 a simple static_cast will return 14. Probably being overly pedantic...and
 * this probably isn't needed but it was easy.
 *
 * @param x number to convert
 * @return ssize_t converted integer type
 */
ssize_t roundi(double x);

/**
 * @brief Deterministically replicate samples.
 *
 * @param samples A list of sample whose weight represents the number of times it needs to be
 * replicated.
 * @return ParticleList of replicated samples with uninitialized weights
 */
ParticleList replication(const ParticleList& samples);
