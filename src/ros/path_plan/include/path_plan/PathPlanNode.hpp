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

  bool set_parameters();

  // Static Methods
private:
  static rclcpp::NodeOptions get_options();

  // Ros 2 pubs and subs
private:
  const rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lidar_data_sub;
  const rclcpp::Subscription<custom_types::msg::Location>::SharedPtr dest_sub;
  const rclcpp::Subscription<custom_types::msg::Location>::SharedPtr location_sub;
  const rclcpp::Publisher<custom_types::msg::Path>::SharedPtr path_pub;
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr weight_map_pub;

  // Node constants
public:
  const bool PARAMETERS_SET;                  // Unread. Used as a cb to setup parameters in constructor
  const double ROBOT_WIDTH;                   // in meters
  const std::pair<double, double> ARENA_SIZE; // in meters
  const double CELL_RESOLUTION;               // 0.01 = 1 sq cm for each cell
  const weight_t TURN_COST;

  // Class Defaults
public:
  static constexpr double DEFAULT_ROBOT_WIDTH = 0.5;                                   // in meters
  static constexpr auto DEFAULT_ARENA_SIZE{std::make_pair<double, double>(6.88, 5.0)}; // in meters
  static constexpr double DEFAULT_CELL_RESOLUTION = 0.01;                              // 0.01 = 1 sq cm for each cell
  static constexpr weight_t DEFAULT_TURN_COST = 10;

  // Node Parameters
public:
  static const std::string BOT_WIDTH_PARAM_NAME;
  static const std::string ARENA_WIDTH_PARAM_NAME;
  static const std::string ARENA_HEIGHT_PARAM_NAME;
  static const std::string CELL_RES_PARAM_NAM;
  static const std::string TURN_COST_PARAM_NAME;

  // mutable Node data
private:
  WeightMap current_map;
  optional_point src;
  optional_point dst;
};
