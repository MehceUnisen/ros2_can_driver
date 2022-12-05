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

set(RECEIVER_HEADER_FILES
        include/ros2_can_driver/can_receiver.h
        )

set(SENDER_HEADER_FILES
       include/ros2_can_driver/can_sender.h
       )

set(RECEIVER_SOURCE_FILES
        src/can_receiver_node.cpp
        src/can_receiver.cpp
        )

set(SENDER_SOURCE_FILES
        src/can_sender.cpp
        src/can_sender_node.cpp
        )

add_executable(ros2_can_receiver_exe
        ${RECEIVER_SOURCE_FILES}
        ${RECEIVER_HEADER_FILES}
        )

add_executable(ros2_can_sender_exe
        ${SENDER_SOURCE_FILES}
        ${SENDER_HEADER_FILES}
        )

ament_target_dependencies(ros2_can_receiver_exe
        rclcpp
        std_msgs
        ros2_can_msgs
        )

ament_target_dependencies(ros2_can_sender_exe
        rclcpp
        std_msgs
        ros2_can_msgs
        )

install(TARGETS
        ros2_can_receiver_exe
        DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS
        ros2_can_sender_exe
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
