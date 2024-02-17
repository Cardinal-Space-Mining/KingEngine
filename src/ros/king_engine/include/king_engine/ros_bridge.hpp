#pragma once

namespace ros_bridge
{
    typedef struct Location
    {
        double x;
        double y;
    } Location;

    /// @brief We are placing this here as a hook to eventually call KingEngineNode::destination_pub::publish. This prevents contamination with the ROS headers (and reduced build times)
    /// @param lco location
    void set_destination(Location loc);

    /// @brief This is a fancy verneer over KingEngineNode::location_sub callback system
    /// @return Location
    Location get_location();
    
} // namespace ros_bridge
