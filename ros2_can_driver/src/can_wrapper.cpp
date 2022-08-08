//
// Created by mehce on 20.07.2022.
//

#include "can_wrapper.h"

using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

using rcl_interfaces::msg::ParameterDescriptor;
using rclcpp::ParameterValue;
using namespace std::chrono_literals;

CanWrapper::CanWrapper() :

        Node("can_wrapper_node"),

        can_dev_{"can0"},

        can_msg_topic_{this->declare_parameter(
                        "gnss_status_topic",
            ParameterValue{"/um482/gnss_status"},
            ParameterDescriptor{})

                .get<std::string>()},

        pub_can_msg_{create_publisher<ros2_can_msgs::msg::CanMessages>(
                can_msg_topic_, rclcpp::QoS{10}, PubAllocT{})},

        timer_{this->create_wall_timer(
                1000ms, std::bind(&CanWrapper::timerCallback, this))}


{
    if(!openSocket()) {
            RCLCPP_INFO(this->get_logger(), "NO DEVICE FOUND ON CAN0 AMK");
    }

    std::thread reader_thread(&CanWrapper::readData, this);
    std::thread sender_thread(&CanWrapper::sendData, this);
    reader_thread.join();
//    sender_thread.join();
}

void CanWrapper::timerCallback() {
//    sendData(0x31, 0x5);
//    readData();
}


bool CanWrapper::openSocket() {

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

bool CanWrapper::closeSocket() {
    if (close(sock_res_) < 0) {
        RCLCPP_INFO(this->get_logger(), "socket closed");
        return false;
    }

    return true;
}

bool CanWrapper::sendData() {
    can_frame_.can_id = can_id;
    can_frame_.can_dlc = can_dlc;

    sprintf(reinterpret_cast<char *>(can_frame_.data), "annen");

    if (write(sock_res_, &can_frame_, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return true;
    }


    return true;
}

bool CanWrapper::readData() {
    while(1) {
        int nbytes = read(sock_res_, &can_frame_, sizeof(struct can_frame));

        if (nbytes < 0) {
            perror("Read");
            return false;
        }

        printf("0x%03X [%d] ",can_frame_.can_id, can_frame_.can_dlc);

        for (int i = 0; i < can_frame_.can_dlc; i++)
            printf("%02X ",can_frame_.data[i]);

        printf("\r\n");
    }

    return true;
}

