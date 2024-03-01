cmake_minimum_required(VERSION 3.8)
project(lidar)



# static stuff
if(${CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT})
  set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install" CACHE PATH "" FORCE)
  set(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT false)    # apparently this overrides the subdirectory whereas just changing the prefix alone does not
  message(STATUS "[KE-LIDAR]: Install prefix overridden: ${CMAKE_INSTALL_PREFIX}")
endif()
if(NOT MSVC)
	if(NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE Release)
		message(STATUS "[KE-LIDAR]: Release build type applied by default for non-MSVC build system")
	endif()
endif()



# build subproject protobuf compiler and install
set(SUBPROJECT_PROTOBUF_BUILD_DIR ${CMAKE_BINARY_DIR}/subproject-protobuf)
message(STATUS "[KE-LIDAR]: Configuring submodule protobuf...")
execute_process(
  COMMAND cmake -S ${PROJECT_SOURCE_DIR}/Sick-Perception
                -B ${SUBPROJECT_PROTOBUF_BUILD_DIR}
                -Dsickperception_MULTISTAGE_STEP=0
                -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
message(STATUS "[KE-LIDAR]: Building and installing submodule protobuf...")
execute_process(
  COMMAND cmake --build ${SUBPROJECT_PROTOBUF_BUILD_DIR} --target install --config Release ${SUBPROJECT_PROTOBUF_PARALLEL_ARG}
)
# configure subproject main lib
message(STATUS "[KE-LIDAR]: Configuring submodule main library...")
set(_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
set(sickperception_BUILD_TEST OFF)
set(sickperception_MULTISTAGE_STEP 1)
add_subdirectory(Sick-Perception)
set(BUILD_SHARED_LIBS ${_BUILD_SHARED_LIBS})





# main build
option(TEST_NOROS "Test build w/o ROS" OFF)   # manually toggle this during tests
if(${TEST_NOROS})
# Test build

  message(STATUS "[KE-LIDAR]: Configuring TEST (NOROS) build")

  add_executable(tmain "src/test/tmain.cpp")
  target_link_libraries(tmain SickPerception_lib)
  install(TARGETS tmain
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  )


else()
# Actual build for project

  message(STATUS "[KE-LIDAR]: Configuring ROS build")

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

  add_executable(main "src/main.cpp" "src/lidar.cpp")
  ament_target_dependencies(main rclcpp custom_types)

  target_link_libraries(main SickPerception_lib)

  target_include_directories(main PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  # IMPORTANT: we have to be really careful with this since the internal lib
  # uses C++20, so for example not using any standard library structs in the
  # lib interface whose implementation changed between C++17 and C++20 >>
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

endif()
