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
    void on_pose_update(const custom_types::msg::Pose& pose);       // timestamp!?
    /** Access the obstacle weightmap from the processing instance */
    void export_map(const custom_types::msg::Map& map);


} // namespace ros_bridge
