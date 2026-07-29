#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "my_epuck_cooperative_exploration/coordinator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(my_epuck_cooperative_exploration::make_coordinator());
  rclcpp::shutdown();
  return 0;
}
