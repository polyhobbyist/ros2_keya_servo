cmake_minimum_required(VERSION 3.5)
project(ros2_keya_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Arduino code can have unused parameters
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

add_executable(${PROJECT_NAME} 
  src/main.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(
  include/${PROJECT_NAME}
  ${SOURCE_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  std_msgs
  ros2_socketcan
  can_msgs)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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