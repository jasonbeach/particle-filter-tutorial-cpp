#pragma once

#include "particle_filter_tutorial_cpp/simulator/simulator.world.hpp"

using Measurement = Eigen::Vector2d;
using MeasurementList = std::vector<Measurement>;

class Robot {
 public:
  struct Params {
    double std_forward = 0.0;
    double std_turn = 0.0;
    double std_meas_distance = 0.0;
    double std_meas_angle = 0.0;
  };

  Robot() = default;
  Robot(const Eigen::Vector3d& initial_state, const Params& params);

  /**
   * @brief Move the robot according to given arguments and within world of given dimensions. The
   * true motion is the sum of the desired motion and additive Gaussian noise that represents the
   * fact that the desired motion cannot exactly be realized, e.g., due to imperfect control and
   * sensing.
   *
   * --- Suggestion for alternative implementation ------------------------------------------------
   * An alternative approach could be to replace the argument names of this function to
   * true_distance_driven and true_rotation. These true motion could be applied to the robot state
   * without adding noise: self.theta += true_rotation self.x += true_distance_driven *
   * np.cos(self.theta) self.y += true_distance_driven * np.sin(self.theta) Then robot displacement
   * measurements can be modelled as follows: measured_distance_driven =
   * self._get_gaussian_noise_sample(desired_distance, self.std_forward) measured_angle_rotated =
   * self._get_gaussian_noise_sample(desired_rotation, self.std_turn) And one can return noisy
   * measurements: return [measured_distance_driven, measured_angle_rotated] that are then being
   * used as input for the particle filter propagation step. This would obviously still require the
   * cyclic world checks (on both measurement and true robot state). The particle filter estimation
   * results will roughly be the same in terms of performance, however, this might be more intuitive
   * for some of the readers.
   * ----------------------------------------------------------------------------------------------
   *
   * @param desired_distance forward motion setpoint of the robot (m)
   * @param desired_rotation angular rotation setpoint of the robot (rad)
   * @param world dimensions of the cyclic world in which the robot executes its motion
   */
  void move(double desired_distance, double desired_rotation, const World& world);

  /**
   * @brief Perform a measurement. The robot is assumed to measure the distance to and angles with
   * respect to all landmarks in meters and radians respectively. While doing so, the robot
   * experiences zero mean additive Gaussian noise.
   *
   * @param world World containing the landmark positions.
   * @return MeasurementList [[dist_to_landmark1, angle_wrt_landmark1], dist_to_landmark2,
   * angle_wrt_landmark2], ...]
   */
  MeasurementList measure(const World& world) const;

  const Eigen::Vector3d& state() const;

 private:
  /**
   * @brief Get a random sample from a 1D Gaussian distribution with mean mu and standard deviation
   * sigma.
   *
   * @param mu mean of distribution
   * @param sigma standard deviation
   * @return double random sample from distribution with given parameters
   */
  static double get_gaussian_noise_sample(double mu, double sigma);

  Eigen::Vector3d current_state_;
  Params params_;
};