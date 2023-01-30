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

install(
  TARGETS 
    demo_running_example 
    demo_range_only
    demo_running_example_extended_Kalman_particle_filter
  RUNTIME
    DESTINATION ${PROJECT_INSTALL_BIN_DIR}
    COMPONENT ${PROJECT_NAME}_RunTime
)