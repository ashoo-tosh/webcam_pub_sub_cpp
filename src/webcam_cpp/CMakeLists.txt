cmake_minimum_required(VERSION 3.8)
project(webcam_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------
# Build the executable
# ------------------------------------------------------------------------------
add_executable(webcam_publisher src/webcam_publisher.cpp)
ament_target_dependencies(webcam_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

add_executable(webcam_subscriber src/webcam_subscriber.cpp)
ament_target_dependencies(webcam_subscriber rclcpp sensor_msgs cv_bridge image_transport OpenCV)


# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------
install(TARGETS
  webcam_publisher
  webcam_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing (optional)
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
