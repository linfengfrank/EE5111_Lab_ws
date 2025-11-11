cmake_minimum_required(VERSION 3.8)
project(turtle_coop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# TODO for student
# Specify the executable and its source files
add_executable(turtle_coop src/turtle_coop.cpp)
# Link libraries
ament_target_dependencies(turtle_coop rclcpp geometry_msgs turtlesim)

# Specify the executable and its source files
add_executable(turtle_coop_clamp src/turtle_coop_clamp.cpp)
# Link libraries
ament_target_dependencies(turtle_coop_clamp rclcpp geometry_msgs turtlesim)

# Specify the executable and its source files
add_executable(turtle_coop_finite_time src/turtle_coop_finite_time.cpp)
# Link libraries
ament_target_dependencies(turtle_coop_finite_time rclcpp geometry_msgs turtlesim)

# Specify the executable and its source files
add_executable(turtle_coop_feedforward src/turtle_coop_feedforward.cpp)
# Link libraries
ament_target_dependencies(turtle_coop_feedforward rclcpp geometry_msgs turtlesim)


# TODO for student
# Install executables
install(TARGETS
	turtle_coop
  turtle_coop_clamp
  turtle_coop_finite_time
  turtle_coop_feedforward
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
