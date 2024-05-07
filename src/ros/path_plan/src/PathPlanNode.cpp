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

#include "path_plan/PathPlanNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

using point = std::pair<uint16_t, uint16_t>;
using path = std::vector<point>;

const std::string PathPlanNode::BOT_WIDTH_PARAM_NAME = "robot_width";
const std::string PathPlanNode::ARENA_WIDTH_PARAM_NAME = "arena_width";
const std::string PathPlanNode::ARENA_HEIGHT_PARAM_NAME = "arena_height";
const std::string PathPlanNode::CELL_RES_PARAM_NAM = "cell_resolution";
const std::string PathPlanNode::TURN_COST_PARAM_NAME = "turn_cost";

PathPlanNode::PathPlanNode()
    : Node("path_plan"),
      lidar_data_sub(this->create_subscription<nav_msgs::msg::OccupancyGrid>("lidar_map", 10, std::bind(&PathPlanNode::lidar_change_cb, this, _1))),
      dest_sub(this->create_subscription<custom_types::msg::Location>("destination", 10, std::bind(&PathPlanNode::destination_change_cb, this, _1))),
      location_sub(this->create_subscription<custom_types::msg::Location>("location", 10, std::bind(&PathPlanNode::location_change_cb, this, _1))),
      path_pub(this->create_publisher<custom_types::msg::Path>("path", 10)),
      weight_map_pub(this->create_publisher<nav_msgs::msg::OccupancyGrid>("weight_map", 10)),
      PARAMETERS_SET(this->set_parameters()),
      ROBOT_WIDTH(this->get_parameter(BOT_WIDTH_PARAM_NAME).as_double()),
      ARENA_SIZE(std::make_pair<float, float>(this->get_parameter(ARENA_WIDTH_PARAM_NAME).as_double(), this->get_parameter(ARENA_HEIGHT_PARAM_NAME).as_double())),
      CELL_RESOLUTION(this->get_parameter(CELL_RES_PARAM_NAM).as_double()),
      TURN_COST(this->get_parameter(TURN_COST_PARAM_NAME).as_int()),
      current_map{static_cast<mapsize_t>(ARENA_SIZE.first / CELL_RESOLUTION), static_cast<mapsize_t>(ARENA_SIZE.second / CELL_RESOLUTION)}
{
  init_map();
  publish_map();
  RCLCPP_INFO(this->get_logger(), "Started path_plan node with params {ROBOT_WIDTH: %lf, ARENA_SIZE: (%lf, %lf), CELL_RESOLUTION: %lf, TURN_COST: %d}", ROBOT_WIDTH, ARENA_SIZE.first, ARENA_SIZE.second, CELL_RESOLUTION, TURN_COST);
}

void PathPlanNode::init_map()
{
  const mapsize_t spread_radius = ROBOT_WIDTH / CELL_RESOLUTION;
  current_map.addBorder(spread_radius, WeightMap::getMaxWeight(), BorderPlace::BOTTOM | BorderPlace::LEFT | BorderPlace::RIGHT | BorderPlace::TOP, true, false);
}

void PathPlanNode::publish_path()
{
  if (!src.has_value() || !dst.has_value())
  {
    RCLCPP_ERROR(this->get_logger(), "Attempted to create path from empty points");
    return;
  }
  auto src_ = src.value();
  auto dst_ = dst.value();

  if (!current_map.isValidPoint(src_.first, src_.second) || !current_map.isValidPoint(dst_.first, dst_.second))
  {
    RCLCPP_ERROR(this->get_logger(), "Attempted to create path from out of bounds points. Src: (%i, %i). Dst: (%i, %i)", src_.first, src_.second, dst_.first, dst_.second);
    return;
  }

  path p;
  try
  {
    p = current_map.getPath(src_.first, src_.second, dst_.first, dst_.second, TURN_COST);
  }
  catch (const std::invalid_argument &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Exception thrown when recalculating path. Error Message: %s", e.what());
    return;
  }

  custom_types::msg::Path ros_path;
  ros_path.path.resize(p.size());

  for (size_t i = 0; i < p.size(); i++)
  {
    ros_path.path[i].x = p[i].first * CELL_RESOLUTION;
    ros_path.path[i].y = p[i].second * CELL_RESOLUTION;
  }
  this->path_pub->publish(ros_path);
}

void PathPlanNode::publish_map()
{
  auto msg = this->get_occupancy_grid();

  this->weight_map_pub->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Published OccupancyGrid");
}

void PathPlanNode::lidar_change_cb(const nav_msgs::msg::OccupancyGrid &map)
{
  RCLCPP_INFO(this->get_logger(), "%s", "Recieved lidar data");
  const mapsize_t spread_radius = ROBOT_WIDTH / CELL_RESOLUTION;
  current_map.spreadDataArray(map.data.data(), map.info.origin.position.x, map.info.origin.position.y, map.info.width, map.info.height, spread_radius);

  this->publish_path();
  this->publish_map();
}

point PathPlanNode::doubles_to_point(double x, double y) const
{
  const int x_translated = x / CELL_RESOLUTION;
  const int y_translated = y / CELL_RESOLUTION;

  return std::make_pair<mapsize_t, mapsize_t>(static_cast<mapsize_t>(x_translated),
                                              static_cast<mapsize_t>(y_translated));
}

void PathPlanNode::location_change_cb(const custom_types::msg::Location &msg)
{
  RCLCPP_INFO(this->get_logger(), "Recieved location: (%F, %F)", msg.x, msg.y);
  auto new_point = this->doubles_to_point(msg.x, msg.y);
  if (current_map.isValidPoint(new_point.first, new_point.second))
  {
    this->src = new_point;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Recieved destination (%i, %i) is out of bounds!", new_point.first, new_point.second);
  }
  this->publish_path();
}

void PathPlanNode::destination_change_cb(const custom_types::msg::Location &msg)
{
  RCLCPP_INFO(this->get_logger(), "Recieved destination: (%F, %F)", msg.x, msg.y);
  auto new_point = this->doubles_to_point(msg.x, msg.y);
  if (current_map.isValidPoint(new_point.first, new_point.second))
  {
    this->src = new_point;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Recieved destination (%i, %i) is out of bounds!", new_point.first, new_point.second);
  }
  this->publish_path();
}

nav_msgs::msg::OccupancyGrid PathPlanNode::get_occupancy_grid()
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = CELL_RESOLUTION;

  mapsize_t w = current_map.getWidth();
  mapsize_t h = current_map.getHeight();
  msg.info.width = w;
  msg.info.height = h;

  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(w * h, 0);

  auto weights = current_map.getWeights();
  for (size_t i = 0; i < w * h; i++)
    msg.data[i] = weights[i] * (100.0f / current_map.getMaxWeight());

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = this->get_clock()->now();

  return msg;
}

bool PathPlanNode::set_parameters()
{
  this->declare_parameter<decltype(PathPlanNode::ROBOT_WIDTH)>(BOT_WIDTH_PARAM_NAME, DEFAULT_ROBOT_WIDTH);
  this->declare_parameter<decltype(PathPlanNode::ARENA_SIZE.first)>(ARENA_WIDTH_PARAM_NAME, DEFAULT_ARENA_SIZE.first);
  this->declare_parameter<decltype(PathPlanNode::ARENA_SIZE.second)>(ARENA_HEIGHT_PARAM_NAME, DEFAULT_ARENA_SIZE.second);
  this->declare_parameter<decltype(PathPlanNode::CELL_RESOLUTION)>(CELL_RES_PARAM_NAM, DEFAULT_CELL_RESOLUTION);
  this->declare_parameter<decltype(PathPlanNode::TURN_COST)>(TURN_COST_PARAM_NAME, DEFAULT_TURN_COST);
  return true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanNode>());
  rclcpp::shutdown();

  return 0;
}