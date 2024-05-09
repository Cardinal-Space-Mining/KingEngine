#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "custom_types/msg/location.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

std::queue<Location> offload_locations({{4.65, 0}, {4.65, 1}, {5.035, 0}, {5.035, 1}, {5.42, 0}, {5.42, 1}, {5.805, 0}, {5.805, 1}, {6.19, 0}, {6.19, 1}, {6.575, 0}, {6.575, 1}});

std::queue<Location> mining_locations({ // needs changing
    {4.65, 0},
    {4.65, 1},
    {5.035, 0},
    { 5.035, 1},
    { 5.42, 0},
    { 5.42, 1},
    { 5.805, 0},
    { 5.805, 1},
    { 6.19, 0},
    { 6.19, 1},
    { 6.575, 0},
    { 6.575,  1}});

bool mining = true;

class KingEngineNode : public rclcpp::Node
{
public:
  KingEngineNode() : Node("king_engine")
  {
    location_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("location", 10, std::bind(&KingEngineNode::location_change_cb, this, _1));
    destination_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("destination", 10);
    // path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  }
  void location_change_cb(const geometry_msgs::msg::PoseStamped &msg)
  {
    bool close_enough = false;
    if (close_enough){
      Location temp_dest;
      if(mining){
        //execute mining

        temp_dest = mining_locations.pop();
        mining = false
      }
      else{
        //execute offload

        temp_dest = offload_locations.pop();
        mining = true
      }
      geometry_msgs::msg::PoseStamped next_destination;
      next_destination = ///convert to pose
      destination_pub->publish(next_destination);
    }
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr location_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr destination_pub;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KingEngineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}