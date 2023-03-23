add_subdirectory(particle_filter)
add_subdirectory(resampling)
add_subdirectory(simulator)

add_executable(demo_running_example
  particle_filter.sir.main.cpp
)

target_link_libraries(
    demo_running_example
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

add_executable(demo_range_only
  particle_filter.range_only.main.cpp
)

target_link_libraries(
    demo_range_only
  PRIVATE
    pf::particle_filter_tutorial_cpp
)


add_executable(demo_running_example_extended_Kalman_particle_filter
  particle_filter.kalman.main.cpp
)

target_link_libraries(
    demo_running_example_extended_Kalman_particle_filter
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

add_executable(demo_running_example_max_weight_resampling
  particle_filter.max_weight_resampling.main.cpp
)

target_link_libraries(
    demo_running_example_max_weight_resampling
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

add_executable(demo_running_example_nepr
  particle_filter.nepr.main.cpp
)

target_link_libraries(
    demo_running_example_nepr
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

add_executable(challenge1_compare_resampling_alorithms
  challenge1_compare_resampling_alorithms.main.cpp
)

target_link_libraries(
    challenge1_compare_resampling_alorithms
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

add_executable(challenge1_compare_resampling_schemes
  challenge1_compare_resampling_schemes.main.cpp
)

target_link_libraries(
    challenge1_compare_resampling_schemes
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

add_executable(challenge1_degeneracy
  challenge1_degeneracy.main.cpp
)

target_link_libraries(
    challenge1_degeneracy
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

add_executable(challenge2_impoverishment
  challenge2_impoverishment.main.cpp
)

target_link_libraries(
    challenge2_impoverishment
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

add_executable(challenge3_divergence
  challenge3_divergence.main.cpp
)

target_link_libraries(
    challenge3_divergence
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

add_executable(challenge5_adaptive_particle_filter
  challenge5_adaptive_particle_filter.main.cpp
)

target_link_libraries(
    challenge5_adaptive_particle_filter
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

install(
  TARGETS 
    demo_running_example 
    demo_range_only
    demo_running_example_extended_Kalman_particle_filter
    demo_running_example_max_weight_resampling
    demo_running_example_nepr
    challenge1_compare_resampling_alorithms
    challenge1_compare_resampling_schemes
    challenge1_degeneracy
    challenge2_impoverishment
    challenge3_divergence
    challenge5_adaptive_particle_filter
  RUNTIME
    DESTINATION ${PROJECT_INSTALL_DIR}
)