if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  function(add_TrajectoryCollection_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest TrajectoryCollection)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  function(add_TrajectoryCollection_test NAME)
    catkin_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} TrajectoryCollection)
  endfunction()
endif()

set(TrajectoryCollection_gtest_list
  TestFunc
  TestCubicSpline
  TestCubicHermiteSpline
  TestCubicInterpolator
  )

foreach(NAME IN LISTS TrajectoryCollection_gtest_list)
  add_TrajectoryCollection_test(${NAME})
endforeach()
