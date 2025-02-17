if(not USE_ROS2)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  function(add_TrajColl_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest TrajColl)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  find_package(ament_cmake_gtest REQUIRED)
  function(add_TrajColl_test NAME)
    ament_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} TrajColl)
  endfunction()
endif()

set(TrajColl_gtest_list
  TestFunc
  TestCubicSpline
  TestCubicHermiteSpline
  TestCubicInterpolator
  TestBangBangInterpolator
  )

foreach(NAME IN LISTS TrajColl_gtest_list)
  add_TrajColl_test(${NAME})
endforeach()
