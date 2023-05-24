cmake_minimum_required(VERSION 3.5)
project(ftg_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(ackermann src/ackermann.cpp)
ament_target_dependencies(ackermann rclcpp ackermann_msgs)
add_executable(test1 src/test1.cpp)
ament_target_dependencies(test1 rclcpp ackermann_msgs sensor_msgs nav_msgs geometry_msgs tf2)
add_executable(test2 src/test2.cpp)
ament_target_dependencies(test2 rclcpp ackermann_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2)
add_executable(test3 src/test3.cpp)
ament_target_dependencies(test3 rclcpp ackermann_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2)
add_executable(test4 src/test4.cpp)
ament_target_dependencies(test4 rclcpp ackermann_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2)
add_executable(test5 src/test5.cpp)
ament_target_dependencies(test5 rclcpp ackermann_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2)

install(TARGETS
  ackermann
  test1
  test2
  test3
  test4
  test5
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/test1.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
