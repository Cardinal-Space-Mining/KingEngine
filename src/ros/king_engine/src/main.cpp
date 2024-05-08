#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>

#include "king_engine/ros_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "custom_types/msg/location.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace ros_bridge;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

Location cached_location;

class KingEngineNode : public rclcpp::Node {
  public:
    KingEngineNode() : Node("king_engine") {
      location_sub = this->create_subscription<custom_types::msg::Location>("location", 10, std::bind(&KingEngineNode::location_change_cb, this, _1));
      destination_pub = this->create_publisher<custom_types::msg::Location>("destination", 10);
    }
    void location_change_cb(const custom_types::msg::Location& msg) {
      RCLCPP_INFO(this->get_logger(), "Recieved location: (%F, %F)", msg.x, msg.y);
      Location l = {msg.x, msg.y};
      cached_location = l;
    }
    rclcpp::Subscription<custom_types::msg::Location>::SharedPtr location_sub;
    rclcpp::Publisher<custom_types::msg::Location>::SharedPtr destination_pub;
};


std::mutex node_mutex;
std::shared_ptr<KingEngineNode> node{nullptr};


void choose_destinations() {
  // excavation zone
  // berm zone
}





void ros_bridge::set_destination(Location l) {
  //Check if node has been created
  if (!node) {
    std::fprintf(stderr, "%s", "update_destination called before initialization");
    return;
  }
  
  //Assemble message
  custom_types::msg::Location loc;
  loc.x = l.x;
  loc.y = l.y;

  //Increment ref count so no dealloc, lock node, and publish
  std::shared_ptr<KingEngineNode> node_inc_ref = node;
  std::lock_guard<std::mutex> node_lock(node_mutex);
  node_inc_ref->destination_pub->publish(loc);
}


Location ros_bridge::get_location() {
  return cached_location;
}


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  node = std::make_shared<KingEngineNode>();
  rclcpp::spin(node);
  node = nullptr;
  rclcpp::shutdown();
  return 0;
}