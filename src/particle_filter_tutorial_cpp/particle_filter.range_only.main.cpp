#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.range_only.hpp"
#include "particle_filter_tutorial_cpp/simulator/simulator.robot.range_only.hpp"
#include "particle_filter_tutorial_cpp/simulator/simulator.visualizer.hpp"
#include "thread"

using namespace std::chrono_literals;
using LimitsParameters = ParticleFilter<SimpleParticle>::LimitsParameters;
using ProcessNoiseParameters = ParticleFilter<SimpleParticle>::ProcessNoiseParameters;
using MeasurementNoiseParameters = ParticleFilter<SimpleParticle>::MeasurementNoiseParameters;

int main() {
  const World world {{10.0, 10.0}, {{2.5, 2.5}, {7.5, 7.5}}};
  const auto n_time_steps = 10;
  Visualizer::Params vp;
  vp.draw_particle_pose = false;
  Visualizer visualizer {vp};
  visualizer.update_robot_radius(0.2);
  visualizer.update_landmark_size(7.0);

  ////
  // True robot properties (simulator settings)
  ////

  // Setpoint (desired) motion robot
  const double robot_setpoint_motion_forward = 0.25;
  const double robot_setpoint_motion_turn = 0.02;

  // True simulated robot motion is set point plus additive zero mean Gaussian noise with these
  // standard deviation
  const double true_robot_motion_forward_std = 0.005;
  const double true_robot_motion_turn_std = 0.002;

  // Robot measurements are corrupted by measurement noise
  const double true_robot_meas_noise_distance_std = 0.2;

  // Initialize simulated robot
  Eigen::Vector3d initial_state {world.get_size().x() * 0.8, world.get_size().y() / 6.0,
                                 M_PI / 2.0};
  Robot::Params rp;
  rp.std_forward = true_robot_motion_forward_std;
  rp.std_turn = true_robot_motion_turn_std;
  rp.std_meas_distance = true_robot_meas_noise_distance_std;

  RobotRange robot {initial_state, rp};

  ////
  // Particle filter settings
  ////

  const double number_of_particles = 1000;
  LimitsParameters pf_state_limits {0.0, world.get_size().x(), 0.0, world.get_size().y()};

  // Process model noise (zero mean additive Gaussian noise)
  const double motion_model_forward_std = 0.1;
  const double motion_model_turn_std = 0.20;
  ProcessNoiseParameters process_noise {motion_model_forward_std, motion_model_turn_std};

  // Measurement noise (zero mean additive Gaussian noise)
  const double meas_model_distance_std = 0.4;
  // const double meas_model_angle_std = 0.3;
  MeasurementNoiseParameters measurement_noise {meas_model_distance_std, 0.0};

  // Set resampling algorithm used
  ResamplingAlgorithms algorithm = ResamplingAlgorithms::MULTINOMIAL;

  // Initialize SIR particle filter with range only measurements
  ParticleFilterRangeOnly particle_filter {number_of_particles, pf_state_limits, process_noise,
                                           measurement_noise, algorithm};
  particle_filter.initialize_particles_uniform();

  ////
  // Start simulation
  ////
  for (int i = 0; i < n_time_steps; ++i) {
    // Simulate robot motion (required motion will not exactly be achieved)
    robot.move(robot_setpoint_motion_forward, robot_setpoint_motion_turn, world);

    // Simulate measurement
    const auto measurements = robot.measure(world);

    // Update particle filter
    particle_filter.update(robot_setpoint_motion_forward, robot_setpoint_motion_turn, measurements,
                           world.landmarks());

    // Visualization
    visualizer.draw_world(world, robot, particle_filter.particles(), false, "g");
    std::this_thread::sleep_for(500ms);
  }
  matplot::show();
}