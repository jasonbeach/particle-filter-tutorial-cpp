# Note: This CMake is not a great example of using CMake for an installable library. It is not meant 
# to create an installable library that can then be linked to. It's a simple demo of a particle 
# filter. The only reason we "install" things is so they're in a convenient location after being 
# built.
#
# change default install location to live beside the build directory
if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install CACHE PATH "Install path" FORCE)
  message (STATUS "Install prefix not specified. Defaulting to: ${CMAKE_INSTALL_PREFIX}")
endif()

cmake_minimum_required(VERSION 3.20)
project(particle_filter_tutorial_cpp VERSION 0.0.1)

# project wide setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)
set(CMAKE_INSTALL_MESSAGE LAZY)

# Set a default build type if none was specified
# https://blog.kitware.com/cmake-and-the-default-build-type/ 
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()

set(PROJECT_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})

include(FetchContent)

FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master) 

FetchContent_Declare(fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG 9.1.0) 

FetchContent_Declare(eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0) 

message(STATUS "Fetching dependencies")
FetchContent_MakeAvailable(matplotplusplus fmt eigen)
message(STATUS "Finished fetching dependencies")
message (STATUS "Installing built files to ${PROJECT_INSTALL_DIR}")

add_library(${PROJECT_NAME} "") # don't specify sources here...use target_sources in sub dirs
add_library(pf::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# these four lines are a hack to mark Matplot++ and Eigen include directories as SYSTEM so they don't generate
# warnings. fmt is polite and doesn't generate warnings even at high warning levels
get_target_property(matplot_inc Matplot++::matplot INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(matplot SYSTEM INTERFACE ${matplot_inc})
get_target_property(eigen_inc Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${eigen_inc})


# this must come after include dependencies so they don't build with them.
target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wall -Wextra -pedantic -Wcast-align -Wshadow -Wconversion
)

# this adds this directory to the include path be that in a build or installed context
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>
    $<INSTALL_INTERFACE:${PROJECT_INSTALL_INCLUDE_DIR}/..>
)

# Good flags to have once we move to a new compiler
# -Wconversion -Wcast-align -Wshadow -Wnull-dereference
# -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wrestrict -Wold-style-cast
#  -Wuseless-cast -Wjump-misses-init -Wformat=2
option(ENABLE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
if(ENABLE_WARNINGS_AS_ERRORS)
  target_compile_options( ${PROJECT_NAME} PRIVATE "-Werror")
endif()

target_link_libraries(${PROJECT_NAME} 
  PUBLIC 
    fmt::fmt
    Eigen3::Eigen
    Matplot++::matplot
)

#main targets built by this project
add_subdirectory(src/${PROJECT_NAME})

install(TARGETS 
    ${PROJECT_NAME} 
    matplot
  DESTINATION 
    ${PROJECT_INSTALL_DIR}
 
)

