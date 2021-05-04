#include "rclcpp/rclcpp.hpp"
#include "map_server.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}