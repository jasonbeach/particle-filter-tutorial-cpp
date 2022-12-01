#pragma once

#include <vector>

#include "Eigen/Dense"
#include "fmt/core.h"

using MeasurementList = std::vector<Eigen::Vector2d>;
using LandmarkList = std::vector<Eigen::Vector2d>;

struct Particle {
  Particle() = default;
  Particle(double weight_, const Eigen::Vector3d state_);
  // shouldn't hardcode state vector as size 3, but ok for now: x, y, theta
  Eigen::Vector3d state;
  double weight;
};
using ParticleList = std::vector<Particle>;

namespace fmt {
template <>
struct formatter<Particle> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const Particle& p, FormatContext& ctx) {
    return format_to(ctx.out(), "{:.1f},{:.1f},{:.1f} with w:{:.6f}", p.state.x(), p.state.y(),
                     p.state.z() * 180.0 / M_PI, p.weight);
  }
};

}  // namespace fmt