add_library(TrajectoryCollection
  CubicHermiteSpline.cpp
  CubicSpline.cpp
  )
target_link_libraries(TrajectoryCollection PUBLIC SpaceVecAlg::SpaceVecAlg)
if(DEFINED CATKIN_DEVEL_PREFIX)
  target_link_libraries(TrajectoryCollection PUBLIC ${catkin_LIBRARIES})
endif()
target_include_directories(TrajectoryCollection PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(TrajectoryCollection PUBLIC cxx_std_17)

if(BUILD_SHARED_LIBS)
  set_target_properties(TrajectoryCollection PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
endif()

install(TARGETS TrajectoryCollection
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/TrajectoryCollection DESTINATION "${INCLUDE_INSTALL_DIR}")
