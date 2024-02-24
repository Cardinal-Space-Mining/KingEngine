# KingEngine
Makes strategic decisions and makes the robot do things.
It contains ros code for each subsystem to communicate.

# Current Architecture
![Controls Architecture Diagram](https://github.com/Cardinal-Space-Mining/KingEngine/blob/ros2-iron/assets/Architecture2.jpg?raw=true)

# Ros2 Installation instructions
see [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/index.html)

# Idea
Abstract away ros workings so not everyone on the team needs to learn ros.
Functions are implemented in main.cpp
Callbacks have no implementation and will be filled by the subsystem person

Layout as follows:
* King Engine
    * Functions:
        * get_location (Returns last known location)
        * set_destination (Sets destination for the rest of the systems)
    * Callbacks:
        * None
* Lidar
    * Functions:
        * set_map (Sends a map to pathplan)
    * Callbacks
        * on_location_update (Triggers whenever localization pushes a location update)
        * on_startup (Runs once at startup. Should return)
* Localization
    * localization_main (a entrypoint that passes a lambda to set location)
* Path Plan
    * Functions:
        * None
    * Callbacks:
        * on_lidar_data (called whenever the lidar sends data)
        * on_location_change (Triggers whenever localization pushes a location update)
        * on_destination_change (Triggers whenever king engine pushes a change of destination)
* Traversal
    * Functions:
        * None
    * Callbacks:
        * on_location_change (Triggers whenever localization pushes a location update)
        * on_path_change (Triggers whenever path plan triggers a new path)

