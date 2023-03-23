

#add_library(simulator
target_sources(${PROJECT_NAME}
  PRIVATE
    simulator.robot.cpp
    simulator.robot.range_only.cpp
    simulator.visualizer.cpp
    simulator.world.cpp
)
