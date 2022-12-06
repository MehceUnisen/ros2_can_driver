//
// Created by mehce on 03.12.2022.
//

#include "can_sender.h"

using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;

using namespace std;
using namespace chrono_literals;
using rcl_interfaces::msg::ParameterDescriptor;
using rclcpp::ParameterValue;

CanSender::CanSender() :

        Node("can_sender_node"),

        can_dev_{"can0"},

        can_send_topic_{this->declare_parameter(
                        "can_send_topic_",
            ParameterValue{"/can_driver/send_frame"},
            ParameterDescriptor{})
                .get<std::string>()},

        sub_send_frame_{create_subscription<ros2_can_msgs::msg::Frame>(
                can_send_topic_, rclcpp::QoS{10},
                std::bind(&CanSender::receiveFrameCallback, this, placeholders::_1),
                           SubAllocT{})},

        timer_{this->create_wall_timer(
                1000ms, std::bind(&CanSender::timerCallback, this))}


{
    if(!openSocket()) {
      RCLCPP_INFO(this->get_logger(), "Can't connect to CAN device\n");
      exit(-1);
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Connected to CAN device\n");
    }
}

void CanSender::timerCallback() {
    std::cout << "Timer Callback\n";
}

bool CanSender::openSocket() {
    if ((sock_res_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        RCLCPP_INFO(this->get_logger(), "socket");
        return false;
    }

    strcpy(ifr_.ifr_name, can_dev_ );
    ioctl(sock_res_, SIOCGIFINDEX, &ifr_);

    memset(&sock_addr_can_, 0, sizeof(sock_addr_can_));
    sock_addr_can_.can_family = AF_CAN;
    sock_addr_can_.can_ifindex = ifr_.ifr_ifindex;

    if (bind(sock_res_, (struct sockaddr *)&sock_addr_can_, sizeof(sock_addr_can_)) < 0) {
        RCLCPP_INFO(this->get_logger(), "bind");
        return false;
    }
    return true;
}

bool CanSender::closeSocket() {
    if (close(sock_res_) < 0) {
        RCLCPP_INFO(this->get_logger(), "socket closed");
        return false;
    }
    return true;
}

bool CanSender::sendData() {

    if (write(sock_res_, &can_frame_, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return false;
    }
    return true;
}

void CanSender::receiveFrameCallback(ros2_can_msgs::msg::Frame::ConstSharedPtr msg)
{
    msg_can_frame_ = msg;
//    can_id = static_cast<int>(msg_can_frame_->id);
//    can_dlc = static_cast<uint8_t>(msg_can_frame_->dlc);
    can_frame_.can_id = msg_can_frame_->id;
    can_frame_.can_dlc = msg_can_frame_->dlc;

    for (int i = 0; i < 8; ++i) {
        can_frame_.data[i] = msg_can_frame_->data[i];
    }


    if(!sendData()) {
        RCLCPP_WARN(this->get_logger(), "Error sending data\n");
    }
    else{
//        RCLCPP_WARN(this->get_logger(), "SSSSSSSSSSending CanId data\n");

    }
}