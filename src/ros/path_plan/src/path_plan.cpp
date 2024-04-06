#include "path_plan/ros_bridge.hpp"
#include "path_plan/WeightMap.hpp"

#include <stdexcept>
#include <limits>

using namespace ros_bridge;

const double ROBOT_WIDTH = 0.5; // in meters
const std::pair<float, float> ARENA_SIZE(6.88f, 5.0f); // in meters
const float CELL_RESOLUTION = 0.01f; // 0.01 = 1 sq cm for each cell


WeightMap current_map(ARENA_SIZE.first / CELL_RESOLUTION, ARENA_SIZE.second / CELL_RESOLUTION);

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

optional_path ros_bridge::on_lidar_data(const custom_types::msg::Map& msg) {
    const mapsize_t spread_radius = ROBOT_WIDTH / CELL_RESOLUTION;

    for (int data_x = 0; data_x < msg.cells_x; data_x++) {
        for (int data_y = 0; data_y < msg.cells_y; data_y++) {
            int map_x = data_x + msg.origin_x;
            int map_y = data_y + msg.origin_y;

            if (!current_map.isValidPoint(map_x, map_y))
                continue;

            int raw_weight = 255 * msg.map[data_x + data_y * msg.cells_x];

            if (!WeightMap::isValidWeight(raw_weight))
                continue;

            current_map.addCircle(map_x, map_y, spread_radius, raw_weight, true, false);
        }
    }

    return update_path();
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
