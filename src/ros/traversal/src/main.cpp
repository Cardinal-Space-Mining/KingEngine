#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>

#include "traversal/ros_bridge.hpp"

#include "rclcpp/rclcpp.hpp"

#include "custom_types/msg/location.hpp"
#include "custom_types/msg/path.hpp"
#include "custom_types/msg/map.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace ros_bridge;


class TraversalNode : public rclcpp::Node
{
  public:

    TraversalNode()
    : Node("traversal")
    {
      location_sub = this->create_subscription<custom_types::msg::Location>("location", 10, std::bind(&TraversalNode::location_change_cb, this, _1));
      path_sub = this->create_subscription<custom_types::msg::Path>("path", 10, std::bind(&TraversalNode::path_change_cb, this, _1));
    }


    void location_change_cb(const custom_types::msg::Location& msg)
    {

      RCLCPP_INFO(this->get_logger(), "Recieved location: (%F, %F)", msg.x, msg.y);
      ros_bridge::on_location_change(msg.x, msg.y);
    
    }

    void path_change_cb(const custom_types::msg::Path& path)
    {
      RCLCPP_INFO(this->get_logger(), "%s","Recieved new path");
      std::vector<std::pair<double,double>> cpath(path.path.size());
      for(size_t i =0; i<path.path.size();i++){
        cpath[i].first = path.path[i].x;
        cpath[i].second = path.path[i].y;
      }
      ros_bridge::on_path_change(cpath);
    }

    rclcpp::Subscription<custom_types::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<custom_types::msg::Location>::SharedPtr location_sub;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversalNode>());
  rclcpp::shutdown();
  return 0;
}