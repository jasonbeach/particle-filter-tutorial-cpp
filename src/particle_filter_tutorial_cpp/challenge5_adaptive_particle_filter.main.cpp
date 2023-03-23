
#include <chrono>
#include <thread>

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.adaptive_kld.hpp"
#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.sir.hpp"
#include "particle_filter_tutorial_cpp/simulator/simulator.visualizer.hpp"

using LimitsParameters = ParticleFilter<SimpleParticle>::LimitsParameters;
using ProcessNoiseParameters = ParticleFilter<SimpleParticle>::ProcessNoiseParameters;
using MeasurementNoiseParameters = ParticleFilter<SimpleParticle>::MeasurementNoiseParameters;
using namespace std::chrono_literals;
using namespace matplot;

/**
 * @brief This file demonstrates the difference between the SIR particle filter that has a constant
 * number of particles and the adaptive particle filter that varies the number of particles on the
 * fly. Afterwards, the number of particles for both particle filters are plotted over time together
 * with the estimation error of the robot's x-position. The results show that the adaptive particle
 * filter achieves a similar estimation accuracy with less particles once the particles have
 * converged.
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

  fmt::print("Starting adaptive particle filter demo.\n");

  World world {{10.0, 10.0}, {{2.0, 2.0}, {2.0, 8.0}, {9.0, 2.0}, {8.0, 9.0}}};

  const size_t n_time_steps = 20;  // Number of simulated time steps

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

  const size_t number_of_particles = 750;
  const auto resampling_algorithm = ResamplingAlgorithms::MULTINOMIAL;

  const LimitsParameters lp {0, world.get_size().x(), 0, world.get_size().y()};
  const ProcessNoiseParameters pnp {0.2, 0.05};  // different than previously?
  // const ProcessNoiseParameters pnp {0.1, 0.20};
  const MeasurementNoiseParameters mnp {0.4, 0.3};

  // (Re)initialize SIR particle filter: resample every time step
  ParticleFilterSIR particle_filter_sir {number_of_particles, lp, pnp, mnp, resampling_algorithm};
  particle_filter_sir.initialize_particles_uniform();

  AdaptiveParticleFilterKld::Parameters p;
  p.resolutions = Eigen::Vector3d {.2, .2, .3};
  p.epsilon = 0.15;
  p.upper_quantile = 3.0;
  p.min_number_of_particles = 50;
  p.max_number_of_particles = 2e4;
  AdaptiveParticleFilterKld particle_filter_kld {number_of_particles, lp, pnp, mnp, p};
  particle_filter_kld.initialize_particles_uniform();

  std::vector<double> errors_kld;
  std::vector<double> npar_kld;
  std::vector<double> errors_sir;
  std::vector<double> npar_sir;
  std::vector<double> time_steps;

  for (int ts = 0; ts < n_time_steps; ++ts) {
    robot.move(robot_setpoint_motion_forward, robot_setpoint_motion_turn, world);
    const auto measurents = robot.measure(world);
    Eigen::Vector3d robot_state = robot.state();

    particle_filter_kld.update(robot_setpoint_motion_forward, robot_setpoint_motion_turn,
                               measurents, world.landmarks());
    // record just the position error
    errors_kld.push_back(
      (robot_state - particle_filter_kld.particles().get_average_state()).head(2).norm());
    npar_kld.push_back(particle_filter_kld.particles().size());

    particle_filter_sir.update(robot_setpoint_motion_forward, robot_setpoint_motion_turn,
                               measurents, world.landmarks());
    errors_sir.push_back(
      (robot_state - particle_filter_sir.particles().get_average_state()).head(2).norm());
    npar_sir.push_back(particle_filter_sir.particles().size());
    //  visualizer.draw_world(world, robot, particle_filter_sir.particles(), true);
    //  std::this_thread::sleep_for(300ms);
    time_steps.push_back(ts);
    fmt::print("adaptive kld num particles: {}\n", particle_filter_kld.particles().size());
  }

  // Show results

  const auto font_size = 18;
  auto h = figure(true);
  h->font_size(font_size);

  tiledlayout(2, 1);
  auto ax1 = nexttile();
  auto l1 = plot(time_steps, errors_sir, "k-");
  hold(on);
  auto l2 = plot(time_steps, errors_kld, "r--");
  ylabel("Position Error (m)");

  auto ax2 = nexttile();
  plot(time_steps, npar_sir, "k-");
  hold(on);
  plot(time_steps, npar_kld, "r--");
  ax2->xlabel("Time Index");
  ax2->ylabel("Number of Particles");

  matplot::legend(ax1, {"Standard", "Adaptive"});

  // plt.yscale('symlog')

  matplot::show();
}