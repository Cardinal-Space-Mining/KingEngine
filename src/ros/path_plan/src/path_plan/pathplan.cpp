#include "path_plan/pathplan.hpp"


using std::placeholders::_1;


PathPlanNode::PathPlanNode(
	double robot_width_m,
	double arena_x_m,
	double arena_y_m,
	double cell_resolution,
	int turn_cost
) :
	Node("path_plan"),
	robot_width{robot_width_m},
	// arena_size{arena_x_m, arena_y_m},
	cell_resolution{cell_resolution},
	turn_cost{turn_cost},
	current_map{
		arena_x_m / cell_resolution,
		arena_y_m / cell_resolution
	}
{
	// subscribers
	lidar_data_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/ldrp/obstacle_grid", 10,
		std::bind(&PathPlanNode::lidar_change_cb, this, _1));
	location_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/uesim/pose", 10,
		std::bind(&PathPlanNode::location_change_cb, this, _1));
	dest_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("destination", 10,
		std::bind(&PathPlanNode::destination_change_cb, this, _1));

	// publishers
	path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 10);
	weight_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("weight_map", 10);

	// add params (for turn cost, arena size)

	// default init
	// this->map_init();	// don't need this
	this->publish_map();
}


void PathPlanNode::map_init() {		// might need to nuke this
	current_map.addBorder(
		(this->robot_width / this->cell_resolution),
		WeightMap::getMaxWeight(),
		(BorderPlace::BOTTOM | BorderPlace::LEFT | BorderPlace::RIGHT | BorderPlace::TOP),
		true,	// gradient
		false	// overwrite
	);
}

PathPlanNode::optional_point PathPlanNode::to_mapsize_ints(double x, double y) {
	const int
		x_translated = static_cast<int>(x / this->cell_resolution),
		y_translated = static_cast<int>(y / this->cell_resolution);

	if (x_translated < 0 || x_translated > std::numeric_limits<uint16_t>::max()) {
		// x doesn't fit into a uint16_t
		return std::nullopt;
	}
	if (y_translated < 0 || y_translated > std::numeric_limits<uint16_t>::max()) {
		// y doesn't fit into a uint16_t
		return std::nullopt;
	}

	return std::make_optional<PathPlanNode::point>(
		static_cast<uint16_t>(x_translated),
		static_cast<uint16_t>(y_translated)
	);
}

PathPlanNode::optional_path PathPlanNode::update_path() {
	if (current_location.has_value() && destination.has_value()) {
		auto src = current_location.value();
		auto dst = current_location.value();
		try {
			return
				current_map.getPath(
					src.first, src.second,
					dst.first, dst.second,
					turn_cost
				);
		} catch (const std::invalid_argument &e) {
			// todo might want to log errors
			return std::nullopt;
		}
	}
	return std::nullopt;
}

void PathPlanNode::publish_path(path& path) {
	nav_msgs::msg::Path ros_path;
	ros_path.poses.resize(path.size());

	for (size_t i = 0; i < path.size(); i++)
	{
		geometry_msgs::msg::PoseStamped& p = ros_path.poses[i];
		p.pose.position.x = static_cast<double>(path[i].first);
		p.pose.position.y = static_cast<double>(path[i].second);
		p.pose.position.z = 0.0;
	}

	this->path_pub->publish(ros_path);
}

void PathPlanNode::publish_map() {
	auto msg = nav_msgs::msg::OccupancyGrid();

	WeightMap::mapsize_t
		_w = current_map.getWidth(),
		_h = current_map.getHeight(),
		_area = _w * _h;

	msg.info.width = _w;
	msg.info.height = _h;
	msg.info.resolution = this->cell_resolution;

	msg.info.origin.position.x = 0.0;
	msg.info.origin.position.y = 0.0;
	msg.info.origin.position.z = 0.0;
	msg.info.origin.orientation.x = 0.0;
	msg.info.origin.orientation.y = 0.0;
	msg.info.origin.orientation.z = 0.0;
	msg.info.origin.orientation.w = 1.0;

	msg.data.resize(_area, 0);

	auto weights = current_map.getWeights();
	for (int i = 0; i < _area; i++)
		msg.data[i] = weights[i] * (100.0f / current_map.getMaxWeight());
	delete[] weights;	// TODO: FIX THIS!!!

	msg.header.stamp = this->get_clock()->now();
	msg.header.frame_id = "world";  // changed this to 'world' so its the same as other nodes

	// msg.info.map_load_time = this->get_clock()->now();

	this->weight_map_pub->publish(msg);

	RCLCPP_INFO(this->get_logger(), "Published map");
}

void PathPlanNode::lidar_change_cb(const nav_msgs::msg::OccupancyGrid& map) {
	RCLCPP_INFO(this->get_logger(), "%s" ,"Recieved lidar data");

	current_map.spreadDataArray(
		map.data.data(),
		map.info.origin.position.x,
		map.info.origin.position.y,
		map.info.width,
		map.info.height,
		map.info.resolution,
		this->cell_resolution,
		(this->robot_width * 0.5 / this->cell_resolution)
	);

	auto path = this->update_path();	// check for time
	if (path.has_value()) {
		this->publish_path(path.value());
	}

	this->publish_map();
}

void PathPlanNode::location_change_cb(const geometry_msgs::msg::PoseStamped& msg) {
	const double
		_x = msg.pose.position.x,
		_y = msg.pose.position.y;

	RCLCPP_INFO(this->get_logger(), "Recieved new location: (%F, %F)", _x, _y);

	// auto path = ros_bridge::on_location_change(_x, _y);
	auto new_location = this->to_mapsize_ints(_x, _y);
	if (new_location.has_value()) current_location = new_location;

	auto path = this->update_path();	// check for time
	if (path.has_value()){
		this->publish_path(path.value());
	}
}

void PathPlanNode::destination_change_cb(const geometry_msgs::msg::PoseStamped& msg) {
	const double
		_x = msg.pose.position.x,
		_y = msg.pose.position.y;

	RCLCPP_INFO(this->get_logger(), "Recieved new destination: (%F, %F)", _x, _y);
	
	// auto path = ros_bridge::on_destination_change(_x, _y);
	auto new_destination = this->to_mapsize_ints(_x, _y);
	if (new_destination.has_value()) current_location = new_destination;

	auto path = this->update_path();	// check for time
	if (path.has_value()){
		this->publish_path(path.value());
	}
}
