cmake_minimum_required(VERSION 3.8)
project(lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_types REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
execute_process(
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/Sick-Perception
  COMMAND cmake . -B ./cmake-build -DMULTISTAGE_STEP=0
  COMMAND cmake --build ./cmake-build --target install --config Release
)
set(BUILD_TEST OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
add_subdirectory(Sick-Perception)

add_executable(main src/main.cpp src/lidar.cpp)
ament_target_dependencies(main rclcpp custom_types)

target_link_libraries(main PRIVATE SickPerception_lib)  # need to setup header export, or just manually include lidar_api.h since that is the only one

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

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
