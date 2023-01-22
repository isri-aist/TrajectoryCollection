if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  function(add_TrajColl_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest TrajColl)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  function(add_TrajColl_test NAME)
    catkin_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} TrajColl)
  endfunction()
endif()

set(TrajColl_gtest_list
  TestFunc
  TestCubicSpline
  TestCubicHermiteSpline
  TestCubicInterpolator
  )

foreach(NAME IN LISTS TrajColl_gtest_list)
  add_TrajColl_test(${NAME})
endforeach()
