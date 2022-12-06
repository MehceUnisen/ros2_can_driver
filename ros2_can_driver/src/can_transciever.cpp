//
// Created by mehce on 06.12.2022.
//


#include "can_transciever.h"

using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;

using namespace std;
using namespace chrono_literals;
using rcl_interfaces::msg::ParameterDescriptor;
using rclcpp::ParameterValue;


std::thread* recvWorker;

CanTransciever::CanTransciever() :

        Node("can_transciever_node"),

        can_dev_{"can0"},

        can_send_topic_{this->declare_parameter(
                        "can_send_topic_",
            ParameterValue{"/can_driver/send_frame"},
            ParameterDescriptor{})
                .get<std::string>()},

        can_recv_topic_{this->declare_parameter(
                 "can_recv_topic_",
                 ParameterValue{"/can_driver/received_frame"},
                 ParameterDescriptor{})
             .get<std::string>()},

        sub_send_frame_{create_subscription<ros2_can_msgs::msg::Frame>(
                can_send_topic_, rclcpp::QoS{10},
                std::bind(&CanTransciever::receiveFrameCallback, this, placeholders::_1),
                           SubAllocT{})},

        pub_recv_frame_{create_publisher<ros2_can_msgs::msg::Frame>(
          can_recv_topic_, rclcpp::QoS{10}, PubAllocT{})},

        timer_{this->create_wall_timer(
                1000ms, std::bind(&CanTransciever::timerCallback, this))}

{

    if(!openSocket()) {
      RCLCPP_INFO(this->get_logger(), "Can't connect to CAN device\n");
      exit(-1);
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Connected to CAN device\n");
    }

  recvWorker = new std::thread(&CanTransciever::readData, this);

}

void CanTransciever::timerCallback() {
//    std::cout << "Timer Callback\n";
// sendData();
}

bool CanTransciever::openSocket()
{
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

bool CanTransciever::closeSocket() {
    if (close(sock_res_) < 0) {
        RCLCPP_INFO(this->get_logger(), "socket closed");
        return false;
    }
    return true;
}

bool CanTransciever::sendData() {

    if (write(sock_res_, &can_frame_, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return false;
    }
    return true;
}

void CanTransciever::receiveFrameCallback(ros2_can_msgs::msg::Frame::ConstSharedPtr msg)
{
    msg_can_send_frame_ = msg;
      can_frame_.can_id = msg_can_send_frame_->id;
      can_frame_.can_dlc = msg_can_send_frame_->dlc;

      for (int i = 0; i < 8; ++i) {
        can_frame_.data[i] = msg_can_send_frame_->data[i];
      }

    if(!sendData()) {
        RCLCPP_WARN(this->get_logger(), "Error sending data\n");
    }
    else{

    }
}

bool CanTransciever::readData() {
  while(1) {
    RCLCPP_INFO(this->get_logger(), "FRAME ID %d", this->can_id);

    int nbytes = read(sock_res_, &can_frame_, sizeof(struct can_frame));
    can_id = can_frame_.can_id & 536870911;
    if (can_id == 0)
      std::cout << 'h';

    if (nbytes < 0) {
      RCLCPP_WARN(this->get_logger(), "READ ERROR");
      publishError();
    }
    if (write(sock_res_, &can_frame_, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
      perror("Write");
      return false;
    }
    publishData();

  }
  return true;
}

void CanTransciever::publishError() {
  msg_can_recv_frame_.header.set__frame_id(static_cast<std::string>(can_recv_topic_));
  msg_can_recv_frame_.header.stamp.set__sec(static_cast<int32_t>(this->get_clock()->now().seconds()));
  msg_can_recv_frame_.header.stamp.set__nanosec(static_cast<uint32_t>(this->get_clock()->now().nanoseconds()));

  msg_can_recv_frame_.set__is_fd_frame(static_cast<bool>(false));
  pub_recv_frame_->publish(msg_can_recv_frame_);
}

void CanTransciever::publishData()
{
  msg_can_recv_frame_.header.set__frame_id(static_cast<std::string>(can_recv_topic_));
  msg_can_recv_frame_.header.stamp.set__sec(static_cast<int32_t>(this->get_clock()->now().seconds()));
  msg_can_recv_frame_.header.stamp.set__nanosec(
    static_cast<uint32_t>(this->get_clock()->now().nanoseconds()));

  msg_can_recv_frame_.set__id(static_cast<uint32_t>(this->can_id));
  msg_can_recv_frame_.set__dlc(static_cast<uint8_t>(can_frame_.can_dlc));
  for (int i = 0; i < 8; ++i) {
    msg_can_recv_frame_.data[i] = can_frame_.data[i];
  }
  msg_can_recv_frame_.set__is_error(static_cast<bool>(false));
  msg_can_recv_frame_.set__is_fd_frame(static_cast<bool>(true));
  pub_recv_frame_->publish(msg_can_recv_frame_);
}