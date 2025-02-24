cmake_minimum_required(VERSION 3.8)
project(demo_aocs_control_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC_FILES
    src/iss_dynamics.cpp
    src/pd_controller.cpp
    src/L_p_func.cpp
    # src/main.cpp
)

add_executable(iss_aocs ${SRC_FILES})
target_include_directories(iss_aocs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(iss_aocs PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  iss_aocs
  "rclcpp"
  "std_srvs"
  "std_msgs"
  "Eigen3"
)

target_link_libraries(iss_aocs Eigen3::Eigen)


install(TARGETS iss_aocs
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
