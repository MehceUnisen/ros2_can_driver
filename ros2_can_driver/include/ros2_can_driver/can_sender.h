//
// Created by mehce on 03.12.2022.
//

#ifndef ROS2_CAN_DRIVER_CAN_SENDER_H
#define ROS2_CAN_DRIVER_CAN_SENDER_H

#include <rclcpp/rclcpp.hpp>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros2_can_msgs/msg/frame.hpp"

class CanSender : public rclcpp::Node {

public:
  CanSender();
  ~CanSender(){
    std::cout << "HOCCAGALIN BEN DE GIDIYOM \n";
    closeSocket();
  }

private:
  int sock_res_;
  int can_id;
  int can_dlc;

  struct sockaddr_can sock_addr_can_;
  struct ifreq ifr_;
  struct can_frame can_frame_;

  const char* can_dev_; //it defined as char* to be able to work with strcpy
  std::string can_send_topic_;

  rclcpp::Subscription<ros2_can_msgs::msg::Frame>::SharedPtr sub_send_frame_;
  rclcpp::TimerBase::SharedPtr timer_;

  uint8_t can_msg_[8] {};

  bool openSocket();
  bool closeSocket();
  bool sendData();
  void timerCallback();
  void receiveFrameCallback(ros2_can_msgs::msg::Frame::ConstSharedPtr msg);

};

#endif  // ROS2_CAN_DRIVER_CAN_SENDER_H