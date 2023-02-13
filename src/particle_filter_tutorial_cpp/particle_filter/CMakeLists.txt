target_sources(${PROJECT_NAME}
  PRIVATE
    particle_filter.adaptive_kld.cpp
    particle_filter.adaptive_sl.cpp
    particle_filter.auxiliary.cpp
    particle_filter.base.cpp
    particle_filter.kalman.cpp
    particle_filter.max_weight_resampling.cpp
    particle_filter.range_only.cpp
    particle_filter.sir.cpp
    particle_filter.types.cpp
)

install(
  FILES 
    particle_filter.adaptive_kld.hpp
    particle_filter.adaptive_sl.hpp
    particle_filter.auxiliary.hpp
    particle_filter.base.hpp
    particle_filter.kalman.cpp
    particle_filter.max_weight_resampling.hpp
    particle_filter.range_only.hpp
    particle_filter.sir.hpp
    particle_filter.types.hpp
  DESTINATION
    ${PROJECT_INSTALL_INCLUDE_DIR}/particle_filter
  COMPONENT 
    ${PROJECT_NAME}_Development
)

# need this for now until we upgrade to newer version of g++ 
# comment out-- not using std::filesystem for now
# set_property(TARGET particle_filter APPEND PROPERTY INTERFACE_LINK_LIBRARIES -lstdc++fs)



if (BUILD_TESTING)
  add_executable(particle_filter_test 
    particle_filter.test.cpp
  )
  target_link_libraries(particle_filter_test 
    PUBLIC 
      catch_main
      pf::particle_filter_tutorial_cpp
  )

  add_test(NAME particle_filter_test COMMAND particle_filter_test )

endif()