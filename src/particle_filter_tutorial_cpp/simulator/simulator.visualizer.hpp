#pragma once

#include <matplot/matplot.h>

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.types.hpp"
#include "particle_filter_tutorial_cpp/simulator/simulator.robot.hpp"
#include "particle_filter_tutorial_cpp/simulator/simulator.world.hpp"

class Visualizer {
 public:
  struct Params {
    double x_margin = 1.0;
    double y_margin = 1.0;
    double circle_radius_robot = 0.02;
    bool draw_particle_pose = false;
    double landmark_size = 6.0;
    double scale = 2.0;
    double robot_arrow_length = 0.5 / scale;
  };
  Visualizer() = default;
  Visualizer(const Params& p);

  void update_robot_radius(double robot_radius);

  void update_landmark_size(double landmark_size);

  void draw_world(const World& world, const Robot& robot, const ParticleList& particles,
                  bool hold_on = false, std::string_view particle_color = "g");

  void add_pose2d(const Eigen::Vector3d& pose, matplot::figure_handle figure,
                  std::string_view color, double radius);

 private:
  Params params_;
  matplot::figure_handle figure_;
  matplot::axes_handle ax_;
};