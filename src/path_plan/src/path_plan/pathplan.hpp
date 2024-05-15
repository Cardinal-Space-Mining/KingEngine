#pragma once

#include "mapping.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"

#include <opencv2/core.hpp>

/** This class represents the path plannig ROS node (duh) */
class PathPlanNode : public rclcpp::Node
{
public:
	PathPlanNode();
	~PathPlanNode() = default;

protected:
	/** Called when obstacle grid gets updated */
	void lidar_change_cb(const nav_msgs::msg::OccupancyGrid &_map);
	/** Called when robot pose from localization gets updated */
	void location_change_cb(const geometry_msgs::msg::PoseStamped &msg);
	/** Called when target pose gets updated */
	void destination_change_cb(const geometry_msgs::msg::PoseStamped &msg);
    /** Called when changing whether we should avoid crossing from the navigation zone to construction zone */
    void avoid_zone_flag_change_cb(const std_msgs::msg::Bool &flag);

    cv::Mat crop_to_arena_size(
            const cv::Mat data,
            const int map_w, const int map_h,
            const int arena_min_x, const int arena_min_y,
            const int arena_width, const int arena_height);

    void globulize();

	/** Rerun navigation through the currently stored map and export the resulting path */
	void export_data();

	bool config_node();

public:
	// everything in meters
	static constexpr double DEFAULT_ROBOT_WIDTH = 0.8;
	static constexpr int DEFAULT_TURN_COST = 0, DEFAULT_MIN_WEIGHT = 25;
	static constexpr double DEFAULT_UPDATE_TIME_S = 0.5;

    static constexpr double DEFAULT_AVOIDANCE_CORNER_X = 3.88;
    static constexpr double DEFAULT_AVOIDANCE_CORNER_Y = 3.00;
    static constexpr int DEFAULT_AVOIDANCE_THICKNESS = 2;
    static constexpr double DEFAULT_ARENA_WIDTH = 6.88;
    static constexpr double DEFAULT_ARENA_HEIGHT = 5.00;

	static const std::string DEFAULT_OUTPUT_FRAME_ID;

	static const std::string ROBOT_WIDTH_PARAM_NAME;
	static const std::string TURN_COST_PARAM_NAME;
	static const std::string MIN_WEIGHT_PARAM_NAME;
	static const std::string UPDATE_TIME_PARAM_NAME;
	static const std::string OUTPUT_FRAME_PARAM_NAME;
    static const std::string AVOID_ZONE_X_PARAM_NAME;
    static const std::string AVOID_ZONE_Y_PARAM_NAME;
	static const std::string AVOID_ZONE_THICKNESS_PARAM_NAME;
    static const std::string ARENA_WIDTH_PARAM_NAME;
    static const std::string ARENA_HEIGHT_PARAM_NAME;

	// Constant state
protected:
	const bool node_init;

	const rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lidar_data_sub;
	const rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dest_sub;
	const rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr location_sub;
    const rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr avoidance_zone_flag_sub;

    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr weight_map_pub;
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raycast_pub;

	const float robot_width;
    const int turn_cost, min_weight;
	const float avoidance_barrier_x, avoidance_barrier_y;
	const int avoidance_barrier_thickness;
    const double arena_width, arena_height;

	const std::string output_frame_id;

	const rclcpp::TimerBase::SharedPtr periodic_publisher;

	// Mutable state
protected:
	NavMap<int64_t, float> nav_map;

    nav_msgs::msg::OccupancyGrid input_weights, weights;
	geometry_msgs::msg::Pose current_pose, target_pose;

    bool include_avoidance_zone = false;

	bool new_dst = false;
	bool new_map_data = false;

};
