//
// Created by mehce on 20.07.2022.
//

#include "can_receiver.h"

using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

using rcl_interfaces::msg::ParameterDescriptor;
using rclcpp::ParameterValue;
using namespace std::chrono_literals;

std::thread* recvWorker;

CanReceiver::CanReceiver() :

        Node("can_receiver_node"),

        can_dev_{"can0"},

        can_recv_topic_{this->declare_parameter(
                        "can_recv_topic_",
            ParameterValue{"/can_driver/received_frame"},
            ParameterDescriptor{})
                .get<std::string>()},

        pub_recv_frame_{create_publisher<ros2_can_msgs::msg::Frame>(
                can_recv_topic_, rclcpp::QoS{10}, PubAllocT{})},

       
        timer_{this->create_wall_timer(
                1000ms, std::bind(&CanReceiver::timerCallback, this))}


{
    if(!openSocket()) {
            RCLCPP_INFO(this->get_logger(), "NO DEVICE FOUND ON CAN0");
    }

    recvWorker = new std::thread(&CanReceiver::readData, this);

}

void CanReceiver::timerCallback() {
}

bool CanReceiver::openSocket() {

    if ((sock_res_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        RCLCPP_INFO(this->get_logger(), "SOCKET OPENING ERROR");
        publishError();
    }

    strcpy(ifr_.ifr_name, can_dev_ );
    ioctl(sock_res_, SIOCGIFINDEX, &ifr_);

    memset(&sock_addr_can_, 0, sizeof(sock_addr_can_));
    sock_addr_can_.can_family = AF_CAN;
    sock_addr_can_.can_ifindex = ifr_.ifr_ifindex;

    if (bind(sock_res_, (struct sockaddr *)&sock_addr_can_, sizeof(sock_addr_can_)) < 0) {
        RCLCPP_WARN(this->get_logger(), "BIND ERROR");
        publishError();
    }

    return true;
}

bool CanReceiver::closeSocket() {
    if (close(sock_res_) < 0) {
        RCLCPP_WARN(this->get_logger(), "SOCKET CLOSING ERROR");
        publishError();
    }
    return true;
}

bool CanReceiver::readData() {
    while(1) {
//        RCLCPP_INFO(this->get_logger(), "RECEIVED A FRAME");

        int nbytes = read(sock_res_, &can_frame_, sizeof(struct can_frame));
        can_id = can_frame_.can_id & 536870911;
        if (nbytes < 0) {
            RCLCPP_WARN(this->get_logger(), "READ ERROR");
            publishError();
        }
        publishData();
    }

    return true;
}

void CanReceiver::publishError() {
ros2_can_msgs::msg::Frame msg_can_frame_;
    msg_can_frame_.header.set__frame_id(static_cast<std::string>(can_recv_topic_));
    msg_can_frame_.header.stamp.set__sec(static_cast<int32_t>(this->get_clock()->now().seconds()));
    msg_can_frame_.header.stamp.set__nanosec(static_cast<uint32_t>(this->get_clock()->now().nanoseconds()));

     msg_can_frame_.set__is_fd_frame(static_cast<bool>(false));
     pub_recv_frame_->publish(msg_can_frame_);
}

void CanReceiver::publishData() {
  ros2_can_msgs::msg::Frame msg_can_frame_;
    msg_can_frame_.header.set__frame_id(static_cast<std::string>(can_recv_topic_));
    msg_can_frame_.header.stamp.set__sec(static_cast<int32_t>(this->get_clock()->now().seconds()));
    msg_can_frame_.header.stamp.set__nanosec(static_cast<uint32_t>(this->get_clock()->now().nanoseconds()));

    msg_can_frame_.set__id(static_cast<uint32_t>(this->can_id));
    msg_can_frame_.set__dlc(static_cast<uint8_t>(can_frame_.can_dlc));
    for (int i = 0; i < 8; ++i) {
      msg_can_frame_.data[i] = can_frame_.data[i];
    }
    msg_can_frame_.set__is_error(static_cast<bool>(false));
    msg_can_frame_.set__is_fd_frame(static_cast<bool>(true));
    pub_recv_frame_->publish(msg_can_frame_);

}