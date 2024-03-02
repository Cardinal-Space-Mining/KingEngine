#include "path_plan/ros_bridge.hpp"
#include "path_plan/WeightMap.hpp"

#include <stdexcept>
#include <limits>

using namespace ros_bridge;

const double ROBOT_WIDTH = 0.5; // in meters
const std::pair<double, double> ARENA_SIZE(6.88, 5.0); // in meters
constexpr point MAP_DIM(688, 500);
WeightMap current_map(MAP_DIM.first, MAP_DIM.second);

optional_point current_location = std::nullopt;
optional_point destination = std::nullopt;

const int turn_cost = 10;

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
    const double x_translated = x/ARENA_SIZE.first  * current_map.getWidth();
    const double y_translated = y/ARENA_SIZE.second * current_map.getHeight();

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

optional_path ros_bridge::on_lidar_data(const custom_types::msg::Map& msg) {
    for (int x = 0; x < cells_x; x++) {
        for (int y = 0; y < cells_y; y++) {
            double arena_x = msg.origin_x + (x * msg.cell_resolution);
            double arena_y = msg.origin_y + (y * msg.cell_resolution);

            mapsize_t map_x = arena_x / ARENA_SIZE.first * current_map.getWidth();
            mapsize_t map_y = arena_y / ARENA_SIZE.second * current_map.getHeight();

            if (current_map.isValidPoint(map_x, map_y)) {
                current_map.addCircle(map_x, map_y, ROBOT_WIDTH / 2, msg.map[y * msg.cell_x + x], true, false);
            }
        }
    }

    return update_path();
}

optional_path ros_bridge::on_location_change(double x, double y) {
    current_location = doubles_to_mapsize_ints(x, y);
    return update_path();
}

optional_path ros_bridge::on_destination_change(double x, double y) {
    destination = doubles_to_mapsize_ints(x, y);
    return update_path();
}
