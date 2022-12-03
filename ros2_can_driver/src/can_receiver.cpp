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
                        "/can_wrapper/received_frame",
            ParameterValue{"/can_wrapper/received_frame"},
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
  std::cout << "Timer Callback\n";
}


bool CanReceiver::openSocket() {

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

bool CanReceiver::closeSocket() {
    if (close(sock_res_) < 0) {
        RCLCPP_INFO(this->get_logger(), "socket closed");
        return false;
    }

    return true;
}

bool CanReceiver::readData() {
    while(1) {
        int nbytes = read(sock_res_, &can_frame_, sizeof(struct can_frame));

        if (nbytes < 0) {
            perror("Read");
            return false;
        }

        msg_can_frame_.set__id(static_cast<uint32_t>(can_frame_.can_id));
        msg_can_frame_.set__id(static_cast<uint8_t>(can_frame_.can_dlc));

        for (int i = 0; i < 8; ++i) {
          can_msg_[i] = can_frame_.data[i];
        }


        std::memcpy(can_msg_, static_cast<void*>(&msg_can_frame_.data), 8);

        pub_recv_frame_->publish(msg_can_frame_);
        printf("0x%03X [%d] ",can_frame_.can_id, can_frame_.can_dlc);

        for (int i = 0; i < can_frame_.can_dlc; i++)
            printf("%02X ",can_frame_.data[i]);

        printf("\r\n");
    }

    return true;
}

