add_library(TrajColl INTERFACE)
target_link_libraries(TrajColl INTERFACE SpaceVecAlg::SpaceVecAlg)
target_include_directories(TrajColl INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(TrajColl INTERFACE cxx_std_17)

install(TARGETS TrajColl
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/TrajColl DESTINATION include)
