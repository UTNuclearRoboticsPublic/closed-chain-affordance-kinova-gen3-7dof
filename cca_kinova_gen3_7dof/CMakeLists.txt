cmake_minimum_required(VERSION 3.8)

project(cca_kinova_gen3_7dof)

# C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages------------------
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cca_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# ROS service generation
find_package(rosidl_default_generators REQUIRED)

# Non-ROS packages
find_package(Eigen3 REQUIRED)
find_package(affordance_util REQUIRED)
find_package(cc_affordance_planner REQUIRED)


# Generate custom ROS service------------------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanToPose.srv"
  DEPENDENCIES
    geometry_msgs
)


# CCA pose service executable-------------------------------------------------

add_executable(cca_pose_service
  src/cca_pose_service.cpp
)

ament_target_dependencies(cca_pose_service
  rclcpp
  cca_ros
  geometry_msgs
)

target_link_libraries(cca_pose_service
  affordance_util::affordance_util
  cc_affordance_planner::cc_affordance_planner
  Eigen3::Eigen
)

# Link generated service typesupport
rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  cca_pose_service
  "${cpp_typesupport_target}"
)

install(
  TARGETS cca_pose_service
  DESTINATION lib/${PROJECT_NAME}
)

# Original CCA executables---------------------------------------------------

add_executable(
  ${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

add_executable(
  ${PROJECT_NAME}_demo
  src/demo/${PROJECT_NAME}_demo.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  cca_ros
)

ament_target_dependencies(
  ${PROJECT_NAME}_demo
  rclcpp
  cca_ros
)

target_link_libraries(
  ${PROJECT_NAME}_node
  affordance_util::affordance_util
  cc_affordance_planner::cc_affordance_planner
  Eigen3::Eigen
)

target_link_libraries(
  ${PROJECT_NAME}_demo
  affordance_util::affordance_util
  cc_affordance_planner::cc_affordance_planner
  Eigen3::Eigen
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration and launch files--------------------------------------------------

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


# Testing-----------------------------------------------------

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  # Uncomment if there is no copyright/license declaration.
  # set(ament_cmake_copyright_FOUND TRUE)

  # Uncomment if not building inside a Git repository.
  # set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()


# Export dependencies-----------------------------------------------------

ament_export_dependencies(rosidl_default_runtime)

ament_package()