#include "particle_filter_tutorial_cpp/simulator/simulator.world.hpp"

World::World(const Eigen::Vector2d& size, const LandmarkList& landmarks) :
    world_size_ {size}, landmarks_ {landmarks} {}

void World::update_world_size(const Eigen::Vector2d& world_size) { world_size_ = world_size; }

const Eigen::Vector2d& World::get_size() const { return world_size_; }

void World::update_landmarks(const LandmarkList& landmarks) { landmarks_ = landmarks; }

const LandmarkList& World::landmarks() const { return landmarks_; }