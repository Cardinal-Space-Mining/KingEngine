#pragma once
#include <functional>
#include <vector>
namespace ros_bridge
{
    void on_location_update(double x, double y);
    void on_startup();
    void set_map(std::vector<double>& vec);
} // namespace ros_bridge
