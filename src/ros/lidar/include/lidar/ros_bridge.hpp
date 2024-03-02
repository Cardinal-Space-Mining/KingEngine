#pragma once
#include <functional>
#include <vector>
#include "custom_types/msg/location.hpp"
#include "custom_types/msg/map.hpp"

namespace ros_bridge
{
    void on_location_update(const custom_types::msg::Location &loc);
    void on_startup();
    void set_map(const custom_types::msg::Map& map);
} // namespace ros_bridge
