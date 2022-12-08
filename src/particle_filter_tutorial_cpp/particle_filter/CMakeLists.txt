target_sources(${PROJECT_NAME}
  PRIVATE
    particle_filter.types.cpp
    particle_filter.base.cpp
    particle_filter.adaptive_kld.cpp
)

install(
  FILES 
    particle_filter.types.hpp
    particle_filter.base.hpp
    particle_filter.adaptive_kld.hpp
  DESTINATION
    ${PROJECT_INSTALL_INCLUDE_DIR}/particle_filter
  COMPONENT 
    ${PROJECT_NAME}_Development
)

# need this for now until we upgrade to newer version of g++ 
# comment out-- not using std::filesystem for now
# set_property(TARGET particle_filter APPEND PROPERTY INTERFACE_LINK_LIBRARIES -lstdc++fs)

# make the dummy plugin
add_executable(particle_filter_main
  particle_filter.main.cpp
)

target_link_libraries(
    particle_filter_main
  PRIVATE
    pf::particle_filter_tutorial_cpp
)

install(TARGETS particle_filter_main
  LIBRARY 
    DESTINATION ${PROJECT_INSTALL_BIN_DIR}
    COMPONENT ${PROJECT_NAME}_RunTime
)

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