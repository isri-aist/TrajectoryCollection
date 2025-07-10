cmake_minimum_required(VERSION 3.14)

set(PROJECT_NAME trajectory_collection)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/TrajectoryCollection)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)

option(USE_ROS2 "Use ROS2" OFF)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

project(trajectory_collection LANGUAGES CXX)

# SpaceVecAlg
find_package(SpaceVecAlg REQUIRED)

if(USE_ROS2)
  find_package(ament_cmake REQUIRED)
else()
  set(BUILD_TESTING OFF)
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
endif()

add_subdirectory(include)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

if(USE_ROS2)
  ament_export_include_directories(include)
  ament_export_targets(${PROJECT_NAME})
  ament_package()
endif()