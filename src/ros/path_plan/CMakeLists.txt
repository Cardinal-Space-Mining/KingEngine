cmake_minimum_required(VERSION 3.9.4)
project(path_plan)

set (CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)  # keeping this even though its now required
  MESSAGE(STATUS "OpenCV installation found")
  add_compile_definitions(HAVE_OPENCV)
  include_directories(${OpenCV_INCLUDE_DIRS})
else()
  MESSAGE(FATAL_ERROR "OpenCV installation not found")
endif()

add_executable(nav_node src/main.cpp src/path_plan/pathplan.cpp)
ament_target_dependencies(nav_node rclcpp nav_msgs geometry_msgs)

if(OpenCV_FOUND)
  target_link_libraries(nav_node ${OpenCV_LIBS})
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT supported OUTPUT error)

if( supported )
    message(STATUS "IPO / LTO enabled")
    set_property(TARGET nav_node PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
    message(STATUS "IPO / LTO not supported: <${error}>")
endif()

target_include_directories(nav_node
  # PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  PRIVATE ${EIGEN3_INCLUDE_DIR})
target_compile_features(nav_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(
  TARGETS nav_node
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
