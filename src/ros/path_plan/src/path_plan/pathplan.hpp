#pragma once

#include "mapping.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

/** This class represents the path plannig ROS node (duh) */
class PathPlanNode : public rclcpp::Node
{
public:
	// everything in meters
	static constexpr double DEFAULT_ROBOT_WIDTH = 0.8;
	static constexpr int DEFAULT_TURN_COST = 0, DEFAULT_MIN_WEIGHT = 25;

	static const std::string ROBOT_WIDTH_PARAM_NAME;
	static const std::string TURN_COST_PARAM_NAME;
	static const std::string MIN_WEIGHT_PARAM_NAME;

public:
	PathPlanNode();
	~PathPlanNode() = default;

protected:
	/** Called when obstacle grid gets updated */
	void lidar_change_cb(const nav_msgs::msg::OccupancyGrid &map);
	/** Called when robot pose from localization gets updated */
	void location_change_cb(const geometry_msgs::msg::PoseStamped &msg);
	/** Called when target pose gets updated */
	void destination_change_cb(const geometry_msgs::msg::PoseStamped &msg);

	/** Rerun navigation through the currently stored map and export the resulting path */
	void recalc_path_and_export();

	bool init_parameters();

	// Constant state
protected:
	const rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lidar_data_sub;
	const rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dest_sub;
	const rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr location_sub;

	const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
	const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr weight_map_pub;

	const bool param_init;
	const float robot_width;
	const int turn_cost, min_weight;

	//Mutable state
protected:
	NavMap<int64_t, float> nav_map;

	nav_msgs::msg::OccupancyGrid weights;
	geometry_msgs::msg::Pose current_pose, target_pose;
};
