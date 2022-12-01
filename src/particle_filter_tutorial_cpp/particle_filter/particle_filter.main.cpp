#include <fmt/color.h>
#include <fmt/core.h>

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"

int main(int argc, char* argv[]) {
  fmt::print("hello world!\n");
  fmt::print("{}: warning\n", fmt::styled("Warning", fmt::fg(fmt::color::yellow)));
}