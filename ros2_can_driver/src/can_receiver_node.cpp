#include "can_receiver.h"

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanReceiver>());
    rclcpp::shutdown();
    return 0;
}