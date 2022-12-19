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

void Visualizer::draw_world(const World& world, const Robot& robot, const ParticleList& particles,
                            bool hold_on, std::string_view particle_color) {
  const double x_min = -params_.x_margin;
  const double x_max = params_.x_margin + world.get_size().x();
  const double y_min = -params_.y_margin;
  const double y_max = params_.y_margin + world.get_size().y();
  // const double fig_size_x = (x_max - x_min) / params_.scale;
  // const double fig_size_y = (y_max - y_min) / params_.scale;

  // fmt::print("[draw_world] plot boundaries x: [{},{}] y: [{},{}]\n", x_min, x_max, y_min, y_max);
  // fmt::print("[draw_world] figure size: [{},{}]\n", fig_size_x, fig_size_y);

  if (!hold_on) {
    // figure_ = matplot::figure(true);
    // ax_ = figure_->add_axes();
    matplot::gca()->clear();
  }
  // h->size(fig_size_x, fig_size_y);

  ax_->hold(true);
  ax_->plot({0.0, world.get_size().x()}, {0.0, 0.0}, "k-")->line_width(2.0);
  ax_->plot({0.0, 0.0}, {0.0, world.get_size().y()}, "k-")->line_width(2.0);
  ax_->plot({0.0, world.get_size().x()}, {world.get_size().y(), world.get_size().y()}, "k-")
    ->line_width(2.0);
  ax_->plot({world.get_size().x(), world.get_size().x()}, {0.0, world.get_size().y()}, "k-")
    ->line_width(2.0);
  ax_->xlim({x_min, x_max});
  ax_->ylim({y_min, y_max});
  ax_->xticks({});
  ax_->yticks({});
  figure_->name("robot world");
  figure_->draw();

  // figure_->title(fmt::format("{} particles", particles.size()));

  std::vector<double> landmark_x;
  std::vector<double> landmark_y;

  for (const auto& l : world.landmarks()) {
    landmark_x.push_back(l.x());
    landmark_y.push_back(l.y());
  }

  ax_->plot(landmark_x, landmark_y, "bs")->line_width(2.0).marker_size(params_.landmark_size);

  if (params_.draw_particle_pose) {
    const auto radius_scale_factor = particles.size();
    for (const auto& p : particles) {
      add_pose2d(p.state, figure_, "r", radius_scale_factor);
    }
  } else {
    std::vector<double> x_vals;
    x_vals.reserve(particles.size());
    std::vector<double> y_vals;
    y_vals.reserve(particles.size());
    for (const auto& p : particles) {
      x_vals.push_back(p.state.x());
      y_vals.push_back(p.state.y());
    }
    const matplot::line_spec ls {std::string(particle_color) + "."};
    ax_->scatter(x_vals, y_vals)->line_spec(ls).line_width(1.0).marker_size(2.0);
  }

  // add robot pose
  add_pose2d(robot.state(), figure_, "r", params_.circle_radius_robot);
  figure_->draw();
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