#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>

#include "lidar/ros_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "custom_types/msg/location.hpp"
#include "custom_types/msg/map.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace ros_bridge;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

void default_cb(double, double)
{
}

std::function<void(double, double)> location_change_cb = default_cb;

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
      : Node("lidar")
  {
    location_sub = this->create_subscription<custom_types::msg::Location>("location", 10, std::bind(&LidarNode::topic_callback, this, _1));
    map_pub = this->create_publisher<custom_types::msg::Map>("map", 10);
  }

  void topic_callback(const custom_types::msg::Location &msg)
  {

    RCLCPP_INFO(this->get_logger(), "Recieved location: (%F, %F)", msg.x, msg.y);
    location_change_cb(msg.x, msg.y);
  }
  rclcpp::Subscription<custom_types::msg::Location>::SharedPtr location_sub;
  rclcpp::Publisher<custom_types::msg::Map>::SharedPtr map_pub;
};

std::mutex node_mutex;
std::shared_ptr<LidarNode> node{nullptr};

void ros_bridge::set_map(std::vector<double> &l)
{
  // Check if node has been created
  if (!node)
  {
    std::fprintf(stderr, "%s", "update_destination called before initialization");
    return;
  }

  // Assemble message
  custom_types::msg::Map m;
  m.map = l;

  // Increment ref count so no dealloc, lock node, and publish
  std::shared_ptr<LidarNode> node_inc_ref = node;
  std::lock_guard<std::mutex> node_lock(node_mutex);
  node_inc_ref->map_pub->publish(m);
}

void ros_bridge::set_location_update_cb(std::function<void(double, double)> func)
{
  location_change_cb = func;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // TODO Spawn Sam Lidar System
  node = std::make_shared<LidarNode>();
  rclcpp::spin(node);
  node = nullptr;
  rclcpp::shutdown();
  return 0;
}