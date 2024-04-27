#include "./auto.hpp"
AutonomyNode::AutonomyNode() : rclcpp::Node("autonomy_node") 
{

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = std::make_shared<AutonomyNode>();
  auto timer = node->create_timer(std::chrono::seconds(0.05), update);
  rclcpp::spin(node);
  node = nullptr;
  rclcpp::shutdown();
  return 0;
}