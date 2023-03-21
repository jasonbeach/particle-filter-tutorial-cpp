
#include <chrono>
#include <thread>

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.sir.hpp"
#include "particle_filter_tutorial_cpp/simulator/simulator.visualizer.hpp"

using LimitsParameters = ParticleFilter<SimpleParticle>::LimitsParameters;
using ProcessNoiseParameters = ParticleFilter<SimpleParticle>::ProcessNoiseParameters;
using MeasurementNoiseParameters = ParticleFilter<SimpleParticle>::MeasurementNoiseParameters;
using namespace std::chrono_literals;

std::pair<double, double> calc_mean_stdev(const std::vector<double> v);

/**
 * @brief This particle filter demonstrates particle filter sample impoverishment. In order to
 * enforce imporerishment, the process model noise is artificially lowered.
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

  fmt::print("Starting demonstration of particle filter sample impoverishment.\n");

  World world {{10.0, 10.0}, {{2.0, 2.0}, {2.0, 8.0}, {9.0, 2.0}, {8.0, 9.0}}};

  Visualizer::Params vp;
  vp.draw_particle_pose = false;

  Visualizer visualizer {vp};

  visualizer.update_robot_radius(0.2);
  visualizer.update_landmark_size(7);

  const size_t n_time_steps = 25;  // Number of simulated time steps

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
  const MeasurementNoiseParameters mnp {0.4, 0.3};

  // IMPOVERISHMENT: artificially set to unreasonably low value for process model noise
  // Note we are redefining the common settings used in our shared simulation settings file.
  const ProcessNoiseParameters pnp {0.003, 0.003};

  // (Re)initialize SIR particle filter: resample every time step
  ParticleFilterSIR particle_filter_sir {number_of_particles, lp, pnp, mnp, resampling_algorithm};

  particle_filter_sir.initialize_particles_uniform();

  for (int ts = 0; ts < n_time_steps; ++ts) {
    robot.move(robot_setpoint_motion_forward, robot_setpoint_motion_turn, world);
    const auto measurents = robot.measure(world);
    particle_filter_sir.update(robot_setpoint_motion_forward, robot_setpoint_motion_turn,
                               measurents, world.landmarks());
    visualizer.draw_world(world, robot, particle_filter_sir.particles(), true);
    std::this_thread::sleep_for(300ms);
  }

  matplot::show();
}