#pragma once

#include <vector>
#include <optional>
#include <utility>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "custom_types/msg/map.hpp"


namespace ros_bridge
{
    using point = std::pair<uint16_t, uint16_t>;
    using path = std::vector<point>;
    using optional_point = std::optional<point>;
    using optional_path = std::optional<path>;

    void map_init();

    optional_path on_lidar_data(const nav_msgs::msg::OccupancyGrid& msg);

    optional_path on_location_change(double x, double y);
    optional_path on_destination_change(double x, double y);

    nav_msgs::msg::OccupancyGrid get_grid_values();

} // namespace ros_bridge


