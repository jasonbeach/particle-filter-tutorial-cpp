#include <thread>

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.sir.hpp"
#include "particle_filter_tutorial_cpp/simulator/simulator.visualizer.hpp"

using namespace std::chrono_literals;

int main() {
  World world {{10.0, 10.0}, {{2.0, 2.0}, {2.0, 8.0}, {9.0, 2.0}, {8.0, 9.0}}};
  const size_t n_time_steps = 30;

  Visualizer::Params vp;
  vp.draw_particle_pose = false;

  Visualizer visualizer {vp};

  visualizer.update_robot_radius(0.2);
  visualizer.update_landmark_size(7);

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

  const size_t number_of_particles = 1000;
  const ParticleFilter::LimitsParameters lp {0, world.get_size().x(), 0, world.get_size().y()};
  const ParticleFilter::ProcessNoiseParameters pnp {0.1, 0.20};
  const ParticleFilter::MeasurementNoiseParameters mnp {0.4, 0.3};
  const auto resampling_algorithm = ResamplingAlgorithms::MULTINOMIAL;

  ParticleFilterSIR particle_filter_sir {number_of_particles, lp, pnp, mnp, resampling_algorithm};
  particle_filter_sir.initialize_particles_uniform();

  for (size_t ts = 0; ts < n_time_steps; ++ts) {
    robot.move(robot_setpoint_motion_forward, robot_setpoint_motion_turn, world);
    const auto measurements = robot.measure(world);
    particle_filter_sir.update(robot_setpoint_motion_forward, robot_setpoint_motion_turn,
                               measurements, world.landmarks());
    visualizer.draw_world(world, robot, particle_filter_sir.particles());
    std::this_thread::sleep_for(300ms);
  }
  matplot::show();
}