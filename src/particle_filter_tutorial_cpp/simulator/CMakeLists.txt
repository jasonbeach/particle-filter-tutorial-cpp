

#add_library(simulator
target_sources(${PROJECT_NAME}
  PRIVATE
    simulator.robot.cpp
    simulator.visualizer.cpp
    simulator.world.cpp
)

install(
  FILES 
    simulator.robot.hpp
    simulator.visualizer.hpp
    simulator.world.hpp
  DESTINATION
    ${PROJECT_INSTALL_INCLUDE_DIR}/simulator
  COMPONENT 
    ${PROJECT_NAME}_Development
)