
#include <matplot/matplot.h>

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.sir.hpp"

using LimitsParameters = ParticleFilter<SimpleParticle>::LimitsParameters;
using ProcessNoiseParameters = ParticleFilter<SimpleParticle>::ProcessNoiseParameters;
using MeasurementNoiseParameters = ParticleFilter<SimpleParticle>::MeasurementNoiseParameters;

std::pair<double, double> calc_mean_stdev(const std::vector<double> v);

/**
 * @brief This file demonstrates the particle filter degeneracy problem that occurs in case a
 * particle filter never resamples.
 */
int main(int argc, char* argv[]) {
  //
  // True robot properties (simulator settings)
  //

  // Setpoint (desired) motion robot
  const double robot_setpoint_motion_forward = 0.25;
  const double robot_setpoint_motion_turn = 0.02;

  // True simulated robot motion is set point plus additive zero mean Gaussian noise with these
  // standard deviation
  const double true_robot_motion_forward_std = 0.005;
  const double true_robot_motion_turn_std = 0.002;

  // Robot measurements are corrupted by measurement noise
  const double true_robot_meas_noise_distance_std = 0.2;
  const double true_robot_meas_noise_angle_std = 0.05;

  fmt::print("Starting demonstration of particle filter degeneracy.\n");

  World world {{10.0, 10.0}, {{2.0, 2.0}, {2.0, 8.0}, {9.0, 2.0}, {8.0, 9.0}}};

  const size_t n_time_steps = 15;  // Number of simulated time steps

  Eigen::Vector3d initial_state;
  initial_state.x() = world.get_size().x() * 0.75;
  initial_state.y() = world.get_size().y() / 5.0;
  initial_state.z() = M_PI / 2.0;

  Robot::Params rp;
  rp.std_forward = true_robot_motion_forward_std;
  rp.std_turn = true_robot_motion_turn_std;
  rp.std_meas_distance = true_robot_meas_noise_distance_std;
  rp.std_meas_angle = true_robot_meas_noise_angle_std;
  // Initialize simulated robot
  Robot robot {initial_state, rp};

  const size_t number_of_particles = 500;
  const auto resampling_algorithm = ResamplingAlgorithms::MULTINOMIAL;

  const LimitsParameters lp {0, world.get_size().x(), 0, world.get_size().y()};
  const ProcessNoiseParameters pnp {0.1, 0.20};
  const MeasurementNoiseParameters mnp {0.4, 0.3};

  // (Re)initialize SIR particle filter: resample every time step
  ParticleFilterSIR particle_filter_sir {number_of_particles, lp, pnp, mnp, resampling_algorithm};
  // Turn OFF resampling
  particle_filter_sir.set_needs_resampling_function([]() { return false; });
  particle_filter_sir.initialize_particles_uniform();

  std::vector<double> max_weights;
  std::vector<double> time_steps;
  std::generate_n(std::back_inserter(max_weights), n_time_steps, [&]() {
    static int time_step = 0;
    robot.move(robot_setpoint_motion_forward, robot_setpoint_motion_turn, world);
    const auto measurents = robot.measure(world);
    particle_filter_sir.update(robot_setpoint_motion_forward, robot_setpoint_motion_turn,
                               measurents, world.landmarks());
    const auto w_max = particle_filter_sir.particles().get_max_weight();
    fmt::print("Time step: {} max weight: {}\n", time_step, w_max);
    time_steps.push_back(time_step);
    time_step++;
    return w_max;
  });

  // Plot weights as function of time step
  auto figure = matplot::figure(true);
  auto ax = figure->add_axes();
  const auto font_size = 14;
  ax->xlabel("Time index");
  ax->ylabel("Maximum particle weight");
  ax->xlim({0, n_time_steps - 1});
  ax->ylim({0, 1.1});
  ax->x_axis().label_font_size(font_size);
  ax->y_axis().label_font_size(font_size);
  ax->plot(time_steps, max_weights, "k-");

  figure->draw();
  matplot::show();
}