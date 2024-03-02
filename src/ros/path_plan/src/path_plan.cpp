#include "path_plan/ros_bridge.hpp"
#include "path_plan/WeightMap.hpp"

#include <stdexcept>
#include <limits>

using namespace ros_bridge;

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

optional_path ros_bridge::on_lidar_data(const std::vector<double> &vec) {
    //TODO Finish this
    (void) vec;

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
