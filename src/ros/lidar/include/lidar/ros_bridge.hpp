#pragma once

#include <functional>
#include <vector>

#include "custom_types/msg/pose.hpp"
#include "custom_types/msg/map.hpp"


namespace ros_bridge {

    /** Gets called on startup -- starts lidar processing */
    void on_startup();
    /** Gets called on shutdown -- closes lidar processing resources */
    void on_shutdown();

    /** Gets called when new localization data is available */
    void on_pose_update(const custom_types::msg::Pose& pose, uint64_t ts_us);       // timestamp!?
    /** Access the obstacle weightmap from the processing instance */
    void collect_map(custom_types::msg::Map& map);
    /** Wait for new map data or until the specified timeout in milliseconds */
    void wait_next_map(custom_types::msg::Map& map, double timeout_ms);


} // namespace ros_bridge
