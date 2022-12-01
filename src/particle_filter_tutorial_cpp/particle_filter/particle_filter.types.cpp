#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.types.hpp"

Particle::Particle(double weight_, const Eigen::Vector3d state_) : state(state_), weight(weight_) {}