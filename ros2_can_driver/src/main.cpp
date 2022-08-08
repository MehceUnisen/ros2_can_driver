#include "can_wrapper.h"

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanWrapper>());
    rclcpp::shutdown();
    return 0;
}