#pragma once
#include <functional>
#include <vector>

#include "custom_types/msg/location.hpp"
#include "custom_types/msg/map.hpp"


namespace ros_bridge {

    /** Gets called on startup -- starts lidar processing */
    void on_startup();
    /** Gets called on shutdown -- closes lidar processing resources */
    void on_shutdown();

    /** Gets called when new localization data is available */
    void on_location_update(double x, double y);   // need (x,y,z,qx,qy,qz,qw) here OR (xyz*, qxyzw*)
    /** Access the obstacle weightmap from the processing instance */
    void set_map(const custom_types::msg::Map& map);


} // namespace ros_bridge
