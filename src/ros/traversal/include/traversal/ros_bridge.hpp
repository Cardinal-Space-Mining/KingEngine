#pragma once
#include <vector>
#include <utility>
namespace ros_bridge{
    void on_location_change(double x, double y);
    void on_path_change(const std::vector<std::pair<double,double>>& vec);
};