#include "path_plan/ros_bridge.hpp"

using namespace ros_bridge;

optional_path ros_bridge::on_lidar_data(const custom_types::msg::Map& map){
    //TODO Finish this
    (void) map;
    return optional_path();
}

optional_path ros_bridge::on_location_change(double x, double y){
    //TODO Finish this
    (void) x;
    (void) y;
    return optional_path();
}

optional_path ros_bridge::on_destination_change(double x, double y){
    //TODO Finish this
    (void) x;
    (void) y;
    return optional_path();
}