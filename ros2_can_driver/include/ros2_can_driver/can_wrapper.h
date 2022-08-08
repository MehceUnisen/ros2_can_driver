//
// Created by mehce on 20.07.2022.
//

#ifndef ROS2_CAN_DRIVER_CAN_WRAPPER_H
#define ROS2_CAN_DRIVER_CAN_WRAPPER_H

#include <thread>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>


#include "rclcpp/rclcpp.hpp"
#include "ros2_can_msgs/msg/can_messages.hpp"


class CanWrapper : public rclcpp::Node{

public:
    CanWrapper();
    ~CanWrapper() override {
        std::cout << "HOCCAGALIN BEN GIDIYOM\n";
        closeSocket();
    }

    bool openSocket();
    bool closeSocket();
    bool sendData();
    bool readData();
    void timerCallback();

private:
    int sock_res_;
    int can_id;
    int can_dlc;


    struct sockaddr_can sock_addr_can_;
    struct ifreq ifr_;
    struct can_frame can_frame_;

    const char* can_dev_; //it defined as char* to be able to work with strcpy
    std::string can_msg_topic_;

    rclcpp::Publisher<ros2_can_msgs::msg::CanMessages>::SharedPtr pub_can_msg_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<uint8_t> send_data_;
    std::vector<uint8_t> receive_data_;


};


#endif //ROS2_CAN_DRIVER_CAN_WRAPPER_H
