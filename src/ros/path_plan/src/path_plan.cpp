#include "path_plan/ros_bridge.hpp"
#include "path_plan/WeightMap.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <stdexcept>
#include <limits>

using namespace ros_bridge;

static constexpr double ROBOT_WIDTH = 0.5; // in meters
static constexpr std::pair<float, float> ARENA_SIZE(6.88f, 5.0f); // in meters
static constexpr float CELL_RESOLUTION = 0.01f; // 0.01 = 1 sq cm for each cell


WeightMap current_map(ARENA_SIZE.first / CELL_RESOLUTION, ARENA_SIZE.second / CELL_RESOLUTION);

optional_point current_location = std::nullopt;
optional_point destination = std::nullopt;

static constexpr weight_t turn_cost = 10;

void ros_bridge::map_init() {
    const mapsize_t spread_radius = ROBOT_WIDTH / CELL_RESOLUTION;
    current_map.addBorder(spread_radius, WeightMap::getMaxWeight(), BorderPlace::BOTTOM | BorderPlace::LEFT | BorderPlace::RIGHT | BorderPlace::TOP, true, false);
}

optional_path update_path() {
    if (current_location.has_value() && destination.has_value()) {
        auto src = current_location.value();
        auto dst = current_location.value();
        try {
            return current_map.getPath(
                    src.first, src.second,
                    dst.first, dst.second,
                    turn_cost);
        }
        catch (const std::invalid_argument &e) {
            // todo might want to log errors
            return std::nullopt;
        }
    }
    return std::nullopt;
}

optional_point doubles_to_mapsize_ints(double x, double y) {
    const int x_translated = x / CELL_RESOLUTION;
    const int y_translated = y / CELL_RESOLUTION;

    if (x_translated < 0 || x_translated > std::numeric_limits<uint16_t>::max()) {
        // x doesn't fit into a uint16_t
        return std::nullopt;
    }
    if (y_translated < 0 || y_translated > std::numeric_limits<uint16_t>::max()) {
        // y doesn't fit into a uint16_t
        return std::nullopt;
    }
    return std::make_optional<point>(static_cast<uint16_t>(x_translated),
                                     static_cast<uint16_t>(y_translated));
}

optional_path ros_bridge::on_lidar_data(const nav_msgs::msg::OccupancyGrid& msg) {
    const mapsize_t spread_radius = ROBOT_WIDTH / CELL_RESOLUTION;
    current_map.spreadDataArray(msg.data.data(), msg.info.origin.position.x, msg.info.origin.position.y, msg.info.width, msg.info.height, spread_radius);
    return update_path();
}

nav_msgs::msg::OccupancyGrid ros_bridge::get_grid_values() {
    auto msg = nav_msgs::msg::OccupancyGrid();
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

    msg.data.resize(w*h, 0);

    auto weights = current_map.getWeights();
    for (size_t i = 0; i < w*h; i++)
        msg.data[i] = weights[i] * (100.0f / current_map.getMaxWeight());

    return msg;
}

optional_path ros_bridge::on_location_change(double x, double y) {
    auto new_location = doubles_to_mapsize_ints(x, y);
    if (new_location.has_value())
        current_location = new_location;
    return update_path();
}

optional_path ros_bridge::on_destination_change(double x, double y) {
    auto new_destination = doubles_to_mapsize_ints(x, y);
    if (new_destination.has_value())
        current_location = new_destination;
    return update_path();
}


