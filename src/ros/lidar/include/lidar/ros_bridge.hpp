#pragma once
#include <functional>
#include <vector>
namespace ros_bridge
{
    void set_location_update_cb(std::function<void(double, double)> func);
    void set_map(std::vector<double>& vec);
} // namespace ros_bridge
