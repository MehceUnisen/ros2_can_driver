//
// Created by mehce on 06.12.2022.
//

#ifndef ROS2_CAN_DRIVER_CAN_TRANSCIEVER_H
#define ROS2_CAN_DRIVER_CAN_TRANSCIEVER_H

#include <rclcpp/rclcpp.hpp>

#include <cstring>
#include <thread>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros2_can_msgs/msg/frame.hpp"
class CanTransciever: public rclcpp::Node {


public:
  CanTransciever();
  ~CanTransciever() override {
    std::cout << "HOCCAGALIN BEN GIDIYOM\n";
    closeSocket();
  }

private:
  bool openSocket();
  bool closeSocket();
  bool readData();
  bool sendData();
  void timerCallback();
  void publishError();
  void publishData();
  void receiveFrameCallback(ros2_can_msgs::msg::Frame::ConstSharedPtr msg);

  int sock_res_;
  uint32_t can_id;
  int can_dlc;

  struct sockaddr_can sock_addr_can_;
  struct ifreq ifr_;
  struct can_frame can_frame_;

  const char* can_dev_; //it defined as char* to be able to work with strcpy

  std::string can_send_topic_;
  std::string can_recv_topic_;

  ros2_can_msgs::msg::Frame::ConstSharedPtr msg_can_send_frame_;
  ros2_can_msgs::msg::Frame msg_can_recv_frame_;

  rclcpp::Subscription<ros2_can_msgs::msg::Frame>::SharedPtr sub_send_frame_;
  rclcpp::Publisher<ros2_can_msgs::msg::Frame>::SharedPtr pub_recv_frame_;

  uint8_t can_msg_[8] {};

  rclcpp::TimerBase::SharedPtr timer_;

};
#endif  // ROS2_CAN_DRIVER_CAN_TRANSCIEVER_H
