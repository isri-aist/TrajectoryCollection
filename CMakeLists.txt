cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME trajectory_collection)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/TrajectoryCollection)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
set(USE_ROS2 OFF)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(trajectory_collection LANGUAGES CXX)

# SpaceVecAlg
add_project_dependency(SpaceVecAlg REQUIRED)

if(USE_ROS2)
  set(DOXYGEN_HTML_OUTPUT html)

  # Eigen
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})

  find_package(ament_cmake REQUIRED) 

  include_directories(include ${ament_INCLUDE_DIRS})
  link_directories(${ament_LIBRARY_DIRS})

  ament_package()
else()
  set(DOXYGEN_HTML_OUTPUT doxygen-html)
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
