#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>

#include "lidar/ros_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "custom_types/msg/pose.hpp"
#include "custom_types/msg/map.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace ros_bridge;


class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
      : Node("lidar")
  {
    pose_sub = this->create_subscription<custom_types::msg::Pose>("pose", 10, std::bind(&LidarNode::topic_callback, this, _1));
    map_pub = this->create_publisher<custom_types::msg::Map>("lidar_map", 10);
  }

  void topic_callback(const custom_types::msg::Pose &msg)
  {

    RCLCPP_INFO(this->get_logger(), "Recieved location: (%F, %F, %F)", msg.x, msg.y, msg.z);
    ros_bridge::on_pose_update(msg);
  }
  rclcpp::Subscription<custom_types::msg::Pose>::SharedPtr pose_sub;
  rclcpp::Publisher<custom_types::msg::Map>::SharedPtr map_pub;
};

std::mutex node_mutex;
std::shared_ptr<LidarNode> node{nullptr};


void ros_bridge::export_map(const custom_types::msg::Map& map)
{
  // Check if node has been created
  if (!node)
  {
    std::fprintf(stderr, "%s", "update_destination called before initialization");
    return;
  }

  // Increment ref count so no dealloc, lock node, and publish
  std::shared_ptr<LidarNode> node_inc_ref = node;
  std::lock_guard<std::mutex> node_lock(node_mutex);
  node_inc_ref->map_pub->publish(map);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  ros_bridge::on_startup();
  node = std::make_shared<LidarNode>();
  rclcpp::spin(node);
  node = nullptr;
  ros_bridge::on_shutdown();
  rclcpp::shutdown();
  return 0;
}