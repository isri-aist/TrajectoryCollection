add_library(TrajColl INTERFACE)
target_link_libraries(TrajColl INTERFACE SpaceVecAlg::SpaceVecAlg)
if(DEFINED USE_ROS2)
  target_link_libraries(TrajColl INTERFACE ${ament_LIBRARIES})
endif()
target_include_directories(TrajColl INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(TrajColl INTERFACE cxx_std_17)

install(TARGETS TrajColl
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/TrajColl DESTINATION "${INCLUDE_INSTALL_DIR}")
