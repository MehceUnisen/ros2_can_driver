//
// Created by mehce on 20.07.2022.
//

#ifndef ROS2_CAN_DRIVER_CAN_RECEIVER_H
#define ROS2_CAN_DRIVER_CAN_RECEIVER_H

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

class CanReceiver : public rclcpp::Node{

public:
  CanReceiver();
  ~CanReceiver() override {
      std::cout << "HOCCAGALIN BEN GIDIYOM\n";
      closeSocket();
  }

private:
    bool openSocket();
    bool closeSocket();
    bool readData();
    void timerCallback();
    void publishError();
    void publishData();

    int sock_res_;
    uint32_t can_id;
    int can_dlc;

    struct sockaddr_can sock_addr_can_;
    struct ifreq ifr_;
    struct can_frame can_frame_;

    const char* can_dev_; //it defined as char* to be able to work with strcpy
    std::string can_recv_topic_;


    ros2_can_msgs::msg::Frame msg_can_frame_;
    rclcpp::Publisher<ros2_can_msgs::msg::Frame>::SharedPtr pub_recv_frame_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint8_t can_msg_[8];
};

#endif  // ROS2_CAN_DRIVER_CAN_RECEIVER_H