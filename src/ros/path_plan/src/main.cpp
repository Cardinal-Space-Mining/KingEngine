#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>

#include "path_plan/ros_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "custom_types/msg/location.hpp"
#include "custom_types/msg/path.hpp"
#include "custom_types/msg/map.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace ros_bridge;


class PathPlanNode : public rclcpp::Node
{
  public:
    PathPlanNode()
    : Node("path_plan")
    {
      lidar_data_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("lidar_map", 10, std::bind(&PathPlanNode::lidar_change_cb, this, _1));
      location_sub = this->create_subscription<custom_types::msg::Location>("location", 10, std::bind(&PathPlanNode::location_change_cb, this, _1));
      dest_sub = this->create_subscription<custom_types::msg::Location>("destination", 10, std::bind(&PathPlanNode::destination_change_cb, this, _1));
      path_pub = this->create_publisher<custom_types::msg::Path>("path", 10);
      weight_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("weight_map", 10);
      publish_map();
    }

    void publish_path(path& path){
      custom_types::msg::Path ros_path;
      ros_path.path.resize(path.size());

      for (size_t i = 0; i < path.size(); i++)
      {
        ros_path.path[i].x = static_cast<double>(path[i].first);
        ros_path.path[i].y = static_cast<double>(path[i].second);
      }
      this->path_pub->publish(ros_path);
    }

    void publish_map() {
        auto msg = ros_bridge::get_grid_values();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        msg.info.map_load_time = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Published OccupancyGrid");

        this->weight_map_pub->publish(msg);
    }

    void lidar_change_cb(const nav_msgs::msg::OccupancyGrid& map){
      RCLCPP_INFO(this->get_logger(), "%s" ,"Recieved lidar data");
      auto path = ros_bridge::on_lidar_data(map);
      if (path.has_value()){
        this->publish_path(path.value());
      }
      this->publish_map();
    }

    void location_change_cb(const custom_types::msg::Location& msg)
    {
      RCLCPP_INFO(this->get_logger(), "Recieved location: (%F, %F)", msg.x, msg.y);
      auto path = ros_bridge::on_location_change(msg.x, msg.y);
      if (path.has_value()){
        this->publish_path(path.value());
      }
    }

    void destination_change_cb(const custom_types::msg::Location& msg)
    {
      RCLCPP_INFO(this->get_logger(), "Recieved destination: (%F, %F)", msg.x, msg.y);
      auto path = ros_bridge::on_destination_change(msg.x, msg.y);
      if (path.has_value()){
        this->publish_path(path.value());
      }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lidar_data_sub;
    rclcpp::Subscription<custom_types::msg::Location>::SharedPtr dest_sub;
    rclcpp::Subscription<custom_types::msg::Location>::SharedPtr location_sub;
    rclcpp::Publisher<custom_types::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr weight_map_pub;
};

int main(int argc, char * argv[])
{
#ifdef HAVE_OPENCV
    std::cout << "OpenCV was found" << std::endl;
#else
    std::cout << "OpenCV was not found, on_lidar_data() will take a while for a large amount of data." << std::endl;
#endif
    ros_bridge::map_init();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanNode>());
    rclcpp::shutdown();

    return 0;
}