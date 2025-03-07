find_package(GTest REQUIRED)
include(GoogleTest)

set(TrajColl_gtest_list
  TestFunc
  TestCubicSpline
  TestCubicHermiteSpline
  TestCubicInterpolator
  TestBangBangInterpolator
)

function(add_TrajColl_test NAME)
  add_executable(${NAME} src/${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC GTest::gtest TrajColl)
  gtest_discover_tests(${NAME})
endfunction()

foreach(NAME IN LISTS TrajColl_gtest_list)
  add_TrajColl_test(${NAME})
endforeach()
