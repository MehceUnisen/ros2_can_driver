//
// Created by mehce on 03.12.2022.
//

#include <can_sender.h>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanSender>());
  rclcpp::shutdown();
  return 0;
}
