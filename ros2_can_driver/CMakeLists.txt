cmake_minimum_required(VERSION 3.8)
project(ros2_can_driver)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set (CMAKE_CXX_STANDARD 20)


include_directories(include/${PROJECT_NAME})
include_directories(ros2_can_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_can_msgs REQUIRED)

set(HEADER_FILES
        "include/ros2_can_driver/can_wrapper.h"
        )

set(SOURCE_FILES
        src/main.cpp
        src/can_wrapper.cpp
        )

add_executable(ros2_can_driver_exe
        ${SOURCE_FILES}
        )

ament_target_dependencies(ros2_can_driver_exe
        rclcpp
        std_msgs
        ros2_can_msgs
        )

install(TARGETS
        ros2_can_driver_exe
        DESTINATION lib/${PROJECT_NAME}
        )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
