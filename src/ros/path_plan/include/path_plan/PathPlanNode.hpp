#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <mutex>


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "custom_types/msg/location.hpp"
#include "custom_types/msg/path.hpp"
#include "custom_types/msg/map.hpp"

#include "path_plan/WeightMap.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

using point = std::pair<uint16_t, uint16_t>;
using path = std::vector<point>;
using optional_point = std::optional<point>;
using optional_path = std::optional<path>;

class PathPlanNode : public rclcpp::Node
{
public:
  PathPlanNode();

private:

  // If possible, recalculates path and publishes it
  void publish_path();

  // Publishes map 
  void publish_map();

  void lidar_change_cb(const nav_msgs::msg::OccupancyGrid &map);

  void location_change_cb(const custom_types::msg::Location &msg);

  void destination_change_cb(const custom_types::msg::Location &msg);

  // Initializes map with boarder
  void init_map();

  // Converts weightmap to standard ros message
  nav_msgs::msg::OccupancyGrid get_occupancy_grid();

  // Converts a pair of doubles to a point
  point doubles_to_point(double x, double y) const;

private:

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lidar_data_sub;
  rclcpp::Subscription<custom_types::msg::Location>::SharedPtr dest_sub;
  rclcpp::Subscription<custom_types::msg::Location>::SharedPtr location_sub;
  rclcpp::Publisher<custom_types::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr weight_map_pub;
  WeightMap current_map{ static_cast<mapsize_t>(ARENA_SIZE.first / CELL_RESOLUTION), static_cast<mapsize_t>(ARENA_SIZE.second / CELL_RESOLUTION)};
  optional_point src;
  optional_point dst;

public:
  static constexpr double ROBOT_WIDTH = 0.5;                        // in meters
  static constexpr auto ARENA_SIZE{std::make_pair<float, float>(6.88f, 5.0f)}; // in meters
  static constexpr float CELL_RESOLUTION = 0.01f;                   // 0.01 = 1 sq cm for each cell
  static constexpr weight_t TURN_COST = 10;
};
