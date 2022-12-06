//
// Created by mehce on 06.12.2022.
//

#include <can_transciever.h>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTransciever>());
  rclcpp::shutdown();

  return 0;
}
