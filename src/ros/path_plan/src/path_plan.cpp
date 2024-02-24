#include "path_plan/ros_bridge.hpp"
#include "path_plan/WeightMap.hpp"

#include <stdexcept>
#include <optional>

using namespace ros_bridge;

point current_location;
point destination;
WeightMap current_map;

const int turn_cost = 10;
const std::pair<double, double> ARENA_SIZE(6.88, 5.0) // in meters

void initialize_map(const uint16_t map_width, const uint16_t map_height) {
    current_map = WeightMap(map_width, map_height);
    // initialize coordinates with impossible numbers to begin so they won't work before both are properly set.
    current_location(std::numeric_limits<uint16_t>::max(), std::numeric_limits<uint16_t>::max());
    destination(std::numeric_limits<uint16_t>::max(), std::numeric_limits<uint16_t>::max());
}

optional_path update_path() {
    try {
        return {current_map.getPath(
                current_location.first, current_location.second,
                destination.first, destination.second,
                turn_cost)};
    }
    catch (const std::invalid_argument& e) {
        // todo might want to log errors
        return std::nullopt;
    }
}

optional_point doubles_to_mapsize_ints(double x, double y) {
    if (x < 0 || x > std::numeric_limits<uint16_t>::max()) {
        // x doesn't fit into a uint16_t
        return std::nullopt;
    }
    if (y < 0 || y > std::numeric_limits<uint16_t>::max()) {
        // y doesn't fit into a uint16_t
        return std::nullopt;
    }
    point result(0, 0);
    result.first = static_cast<uint16_t>(x/ARENA_SIZE.first * current_map.getWidth());
    result.second = static_cast<uint16_t>(y/ARENA_SIZE.second * current_map.getHeight());
    return std::make_optional(result);
}

optional_path ros_bridge::on_lidar_data(const std::vector<double> &vec) {
    //TODO Finish this
    (void) vec;

    return update_path();
}

optional_path ros_bridge::on_location_change(double x, double y) {
    optional_point new_point = doubles_to_mapsize_ints(x, y);
    if (new_point.hasValue()) {
        current_location = new_point.value();
        return update_path();
    }
    return std::nullopt;
}

optional_path ros_bridge::on_destination_change(double x, double y) {
    optional_point new_point = doubles_to_mapsize_ints(x, y);
    if (new_point.hasValue()) {
        destination = new_point.value();
        return update_path();
    }
    return std::nullopt;
}
