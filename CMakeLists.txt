cmake_minimum_required(VERSION 3.8)
project(lidar_filter_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(dummy_lidar_publisher src/dummy_lidar_publisher.cpp)
ament_target_dependencies(dummy_lidar_publisher rclcpp sensor_msgs)

add_executable(lidar_filter src/lidar_filter.cpp)
ament_target_dependencies(lidar_filter rclcpp sensor_msgs)

target_include_directories(lidar_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(lidar_filter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS dummy_lidar_publisher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS lidar_filter
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz
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