#include <iostream>
#include <vector>
#include "path_plan.hpp" 
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {
    // Define example path
    std::vector<WeightMap::point_t> example_path = {
        {0, 0}, {2, 3}, {5, 6}, {8, 4}, {10, 0}
    };

    // Create profile object with the example path
    profile example_profile(example_path);

    // Visualize the motion profile
    std::vector<double> x_points, y_points;
    for (const auto& point : example_path) {
        x_points.push_back(point.x);
        y_points.push_back(point.y);
    }

    // Plot the path
    plt::plot(x_points, y_points, "bo-");

    // Plot the motion profile
    // (You need to implement methods for motion_path generation in profile class)
    // For simplicity, let's assume linear motion between each pair of points
    for (const auto& motion_node : example_profile.motion_path) {
        auto start_point = motion_node.a;
        auto end_point = motion_node.b;
        plt::plot({ start_point.x, end_point.x }, { start_point.y, end_point.y }, "r-");
    }

    // Add labels and show plot
    plt::title("Motion Profile Visualization");
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::grid(true);
    plt::show();

    return 0;
}