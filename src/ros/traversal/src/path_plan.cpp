#include "traversal/ros_bridge.hpp"

void ros_bridge::on_location_change(double x, double y){
    (void)x;
    (void)y;
    //TODO Finish
}

void ros_bridge::on_path_change(const std::vector<std::pair<double,double>>& vec){
    (void) vec;
    //TODO Finish
}