#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.max_weight_resampling.hpp"
#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.nepr.hpp"
#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.sir.hpp"

using LimitsParameters = ParticleFilter<SimpleParticle>::LimitsParameters;
using ProcessNoiseParameters = ParticleFilter<SimpleParticle>::ProcessNoiseParameters;
using MeasurementNoiseParameters = ParticleFilter<SimpleParticle>::MeasurementNoiseParameters;

std::pair<double, double> calc_mean_stdev(const std::vector<double> v);

/**
 * @brief In this program three particle filter will be used for exactly the same problem. The
 * filters are identical except for the resampling strategy. 1) The first particle filter resamples
 * at every time step (SIR) 2) The second particle filter resamples if the approximate number of
 * effective particles drops below a pre-defined threshold (NEPR) 3) The third particle filter
 * resamples in case the reciprocal of the maximum weight drops below a pre-defined threshold (MWR)
 *
 * @param argc
 * @param argv
 * @return int
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

  fmt::print("Compare three resampling schemes.\n");

  World world {{10.0, 10.0}, {{2.0, 2.0}, {2.0, 8.0}, {9.0, 2.0}, {8.0, 9.0}}};

  Eigen::Vector3d initial_state;
  initial_state.x() = world.get_size().x() * 0.75;
  initial_state.y() = world.get_size().y() / 5.0;
  initial_state.z() = M_PI / 2.0;

  const LimitsParameters lp {0, world.get_size().x(), 0, world.get_size().y()};
  const ProcessNoiseParameters pnp {0.1, 0.20};
  const MeasurementNoiseParameters mnp {0.4, 0.3};

  Robot::Params rp;
  rp.std_forward = true_robot_motion_forward_std;
  rp.std_turn = true_robot_motion_turn_std;
  rp.std_meas_distance = true_robot_meas_noise_distance_std;
  rp.std_meas_angle = true_robot_meas_noise_angle_std;

  const auto resampling_algorithm = ResamplingAlgorithms::MULTINOMIAL;
  const size_t number_of_particles = 1000;
  const double number_of_effective_particles_threshold = number_of_particles / 4.0;
  const double reciprocal_max_weight_resampling_threshold = 1.0 / 0.005;

  /*
   * Simulation settings
   */

  const size_t n_time_steps = 50;  // Number of simulated time steps
  const size_t n_trials = 100;     // Number of times each simulation will be repeated

  // Bookkeeping variables
  std::vector<double> errors_sir;
  std::vector<double> errors_nepr;
  std::vector<double> errors_mwr;
  size_t cnt_sir = 0;
  size_t cnt_nepr = 0;
  size_t cnt_mwr = 0;

  // Start main simulation loop
  for (int trial_cnt = 0; trial_cnt < n_trials; ++trial_cnt) {
    fmt::print("Trial: {}\n", trial_cnt);

    // Initialize simulated robot
    Robot robot {initial_state, rp};

    // (Re)initialize SIR particle filter: resample every time step
    ParticleFilterSIR particle_filter_sir {number_of_particles, lp, pnp, mnp, resampling_algorithm};
    particle_filter_sir.initialize_particles_uniform();

    // Resample if approximate number effective particle drops below threshold
    ParticleFilterNEPR particle_filter_nepr {number_of_particles,
                                             lp,
                                             pnp,
                                             mnp,
                                             resampling_algorithm,
                                             number_of_effective_particles_threshold};
    particle_filter_nepr.set_particles(particle_filter_sir.particles());

    // Resample based on reciprocal of maximum particle weight drops below threshold
    ParticleFilterMWR particle_filter_mwr {number_of_particles,
                                           lp,
                                           pnp,
                                           mnp,
                                           resampling_algorithm,
                                           reciprocal_max_weight_resampling_threshold};
    particle_filter_mwr.set_particles(particle_filter_sir.particles());

    for (int ts = 0; ts < n_time_steps; ++ts) {
      // Move the simulated robot
      robot.move(robot_setpoint_motion_forward, robot_setpoint_motion_turn, world);

      // Simulate measurement
      const auto measurements = robot.measure(world);

      // Update SIR particle filter(in this case : propagate + weight update + resample)
      // res =
      particle_filter_sir.update(robot_setpoint_motion_forward, robot_setpoint_motion_turn,
                                 measurements, world.landmarks());
      //            if res:
      cnt_sir += 1;

      // Update NEPR particle filter(in this case : propagate + weight update, resample if needed)
      // res =
      particle_filter_nepr.update(robot_setpoint_motion_forward, robot_setpoint_motion_turn,
                                  measurements, world.landmarks());
      //            if res:
      cnt_nepr += 1;

      // Update MWR particle filter(in this case : propagate + weight update, resample if needed)
      // res =
      particle_filter_mwr.update(robot_setpoint_motion_forward, robot_setpoint_motion_turn,
                                 measurements, world.landmarks());
      //            if res:
      cnt_mwr += 1;

      // Compute errors
      const Eigen::Vector3d robot_pose = robot.state();
      const Eigen::Vector3d e_sir =
        robot_pose - particle_filter_sir.particles().get_average_state();
      const Eigen::Vector3d e_nepr =
        robot_pose - particle_filter_nepr.particles().get_average_state();
      const Eigen::Vector3d e_mwr =
        robot_pose - particle_filter_mwr.particles().get_average_state();

      errors_sir.push_back(e_sir.norm());
      errors_nepr.push_back(e_nepr.norm());
      errors_mwr.push_back(e_mwr.norm());
    }
  }
  const auto [sm, ss] = calc_mean_stdev(errors_sir);
  fmt::print("SIR mean error: {}, std error: {}\n", sm, ss);

  const auto [nm, ns] = calc_mean_stdev(errors_nepr);
  fmt::print("NEPR mean error: {}, std error: {}\n", nm, ns);

  const auto [mm, ms] = calc_mean_stdev(errors_mwr);
  fmt::print("MWR mean error: {}, std error: {}\n", mm, ms);

  fmt::print("#updates in {} trials: {}, {}, {}\n", n_trials, cnt_sir, cnt_nepr, cnt_mwr);
}

std::pair<double, double> calc_mean_stdev(const std::vector<double> v) {
  if (v.empty()) {
    return {0.0, 0.0};
  }
  const auto count_d = static_cast<double>(v.size());
  const double mean = std::accumulate(v.begin(), v.end(), 0.0) / count_d;
  const double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
  const double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
  return {mean, stdev};
}
