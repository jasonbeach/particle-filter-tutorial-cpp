# Copyright (c) 2020, Scientific Systems Company Inc. (SSCI) All rights reserved.
# See FOCUS Software Licence v1.3
cmake_minimum_required(VERSION 3.20)
#fetch cross compiler and set CMAKE_TOOLCHAIN_FILE here if needed
file(STRINGS ${CMAKE_CURRENT_LIST_DIR}/version project_VER)
message("Configuring particle_filter version ${project_VER}.")
project(particle_filter_tutorial_cpp VERSION ${project_VER})

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# project wide setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)
set(CMAKE_INSTALL_MESSAGE LAZY)

# build tests if we're the top level project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

# Set a default build type if none was specified
# https://blog.kitware.com/cmake-and-the-default-build-type/ 
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()

option(BUILD_SHARED_LIBS "Default to building Shared Libraries" ON)


find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)

if (BUILD_TESTING)
  find_package(Catch2 MODULE REQUIRED)
  include(tests/CMakeLists.txt)
endif()

# include files will go in their own subdirectory, but the libraries themselves
# will get mashed in with all others to avoid having to mess with linker path (by default)
set(PROJECT_INSTALL_BIN_DIR ${CMAKE_INSTALL_BINDIR})
set(PROJECT_INSTALL_LIB_DIR ${CMAKE_INSTALL_LIBDIR})
set(PROJECT_INSTALL_CONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
set(PROJECT_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
set(PROJECT_INSTALL_DATA_DIR ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

add_library(${PROJECT_NAME} "") # don't specify sources here...use target_sources in sub dirs
add_library(pf::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# this must come after include dependencies so they don't build with them.
target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wall -Wextra -pedantic -Wcast-align -Wshadow #-Wconversion
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
)

#main targets built by this project
add_subdirectory(src/${PROJECT_NAME})

set_target_properties(${PROJECT_NAME}
  PROPERTIES 
    SOVERSION ${PROJECT_VERSION_MAJOR}
    VERSION ${PROJECT_VERSION}
)

# Installation Instructions

# install target (e.g. the actual .a or .so files) and create an export set so the library can be 
# imported into another project
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets
  RUNTIME 
  DESTINATION ${PROJECT_INSTALL_BIN_DIR}
    COMPONENT ${PROJECT_NAME}_RunTime
  ARCHIVE 
  DESTINATION ${PROJECT_INSTALL_LIB_DIR}
    COMPONENT ${PROJECT_NAME}_Development
  LIBRARY 
  DESTINATION ${PROJECT_INSTALL_LIB_DIR}
    COMPONENT ${PROJECT_NAME}_RunTime
    NAMELINK_COMPONENT ${PROJECT_NAME}_Development
)

# create the file with the exported set of targets in it and install it
install(
  EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE raptor::
  DESTINATION ${PROJECT_INSTALL_CONFIG_DIR}
  COMPONENT ${PROJECT_NAME}_Development
)

# create a version file with compatibility data
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# create the config file that will find dependencies and load the exported targets for use by a 
# downstream project
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${PROJECT_INSTALL_CONFIG_DIR}
)

install(FILES 
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  DESTINATION ${PROJECT_INSTALL_CONFIG_DIR}
  COMPONENT ${PROJECT_NAME}_Development
)
