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
include(GNUInstallDirs) # For CMAKE_INSTALL_LIBDIR

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
else()
  install(EXPORT ${PROJECT_NAME}
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    )

  include(CMakePackageConfigHelpers)
  # generate the config file that is includes the exports
  configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    )

  # generate the version file for the config file
  # Extract version numbers from package.xml
  file(READ package.xml PACKAGE_XML)
  string(REGEX MATCH "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>" DIRTY_VERSION_STRING ${PACKAGE_XML})
  string(REGEX REPLACE "^<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>$" "\\1" TC_MAJOR_VERSION "${DIRTY_VERSION_STRING}")
  string(REGEX REPLACE "^<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>$" "\\2" TC_MINOR_VERSION "${DIRTY_VERSION_STRING}")
  string(REGEX REPLACE "^<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>$" "\\3" TC_PATCH_VERSION "${DIRTY_VERSION_STRING}")
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${TC_MAJOR_VERSION}.${TC_MINOR_VERSION}.${TC_PATCH_VERSION}"
    COMPATIBILITY AnyNewerVersion
    )

  # install config files
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )
endif()