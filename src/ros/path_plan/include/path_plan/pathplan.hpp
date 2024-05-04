#pragma once

#include <functional>
#include <optional>
#include <utility>
#include <thread>
#include <mutex>

#include "path_plan/WeightMap.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


/** This class represents the path plannig ROS node (duh) */
class PathPlanNode : public rclcpp::Node
{
public:
	using point = std::pair<uint16_t, uint16_t>;
	using path = std::vector<point>;
	using optional_point = std::optional<point>;
	using optional_path = std::optional<path>;

	static constexpr double // everything in meters
		DEFAULT_ROBOT_WIDTH = 0.5,
		DEFAULT_ARENA_X = 6.88,
		DEFAULT_ARENA_Y = 5.0,
		DEFAULT_CELL_RESOLUTION = 0.05;
	static constexpr int
		DEFAULT_TURN_COST = 10;

public:
	PathPlanNode(
		double robot_width_m = DEFAULT_ROBOT_WIDTH,
		double arena_x_m = DEFAULT_ARENA_X,
		double arena_y_m = DEFAULT_ARENA_Y,
		double cell_resolution = DEFAULT_CELL_RESOLUTION,
		int turn_cost = DEFAULT_TURN_COST
	);

	void map_init();
	void publish_path(path& path);
	void publish_map();

	void lidar_change_cb(const nav_msgs::msg::OccupancyGrid& map);
	void location_change_cb(const geometry_msgs::msg::PoseStamped& msg);
	void destination_change_cb(const geometry_msgs::msg::PoseStamped& msg);

protected:
	optional_point to_mapsize_ints(double x, double y);
	optional_path update_path();


protected:
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lidar_data_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dest_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr location_sub;

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr weight_map_pub;

	const double robot_width;
	// const std::pair<double, double> arena_size;
	const double cell_resolution;
	const int turn_cost;

	WeightMap current_map; // needs to be fixed anyway

	optional_point current_location = std::nullopt;
	optional_point destination = std::nullopt;


};
