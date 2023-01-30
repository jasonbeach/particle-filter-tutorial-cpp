
install(
  FILES 
    resampler.helpers.hpp
    resampler.hpp
  DESTINATION
    ${PROJECT_INSTALL_INCLUDE_DIR}/resampling
  COMPONENT 
    ${PROJECT_NAME}_Development
)

# we should add some testing
# if (BUILD_TESTING)
#   add_executable(particle_filter_test 
#     particle_filter.test.cpp
#   )
#   target_link_libraries(particle_filter_test 
#     PUBLIC 
#       catch_main
#       pf::particle_filter
#   )

#   add_test(NAME particle_filter_test COMMAND particle_filter_test )

# endif()