cmake_minimum_required(VERSION 3.8)
project(goleiro_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# --- localization_node (antigo pacote "posicao") ------------------------------
add_executable(localization_node src/localization_node.cpp)
ament_target_dependencies(localization_node rclcpp std_msgs custom_interfaces)

# --- goleiro_behavior_node (antigo pacote "decision") --------------------------
add_executable(goleiro_behavior_node src/goleiro_behavior.cpp)
ament_target_dependencies(goleiro_behavior_node rclcpp rclcpp_action std_msgs custom_interfaces)

install(TARGETS
  localization_node
  goleiro_behavior_node
  DESTINATION lib/${PROJECT_NAME}
)


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
