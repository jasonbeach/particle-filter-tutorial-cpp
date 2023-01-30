#include "particle_filter_tutorial_cpp/simulator/simulator.visualizer.hpp"

Visualizer::Visualizer(const Params& p) :
    params_ {p}, figure_ {matplot::figure(true)}, ax_ {figure_->add_axes()} {}

void Visualizer::update_robot_radius(double robot_radius) {
  params_.circle_radius_robot = robot_radius;
  params_.robot_arrow_length = robot_radius;
}

void Visualizer::update_landmark_size(double landmark_size) {
  params_.landmark_size = landmark_size;
}

void Visualizer::add_pose2d(const Eigen::Vector3d& pose, matplot::figure_handle figure,
                            std::string_view color, double radius) {
  matplot::figure(figure);

  matplot::ellipse(pose.x() - radius, pose.y() - radius, 2.0 * radius, 2.0 * radius)
    ->fill(true)
    .color(color);
  const double x1 = pose.x();
  const double x2 = x1 + radius * cos(pose.z());
  const double y1 = pose.y();
  const double y2 = y1 + radius * sin(pose.z());
  matplot::gca()->plot({x1, x2}, {y1, y2})->color("blue").line_width(2.0);
}