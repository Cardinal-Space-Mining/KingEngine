# KingEngine
Makes strategic decisions and makes the robot do things.
It contains ros code for each subsystem to communicate.

# Current Architecture
![Controls Architecture Diagram](https://github.com/Cardinal-Space-Mining/KingEngine/blob/ros2-iron/assets/Architecture2.jpg?raw=true)

# Ros2 Installation instructions
see https://docs.ros.org/en/iron/Installation.html

# Idea
Abstract away ros workings behind predefined function calls so not everyone on the team needs to learn ros.
These calls are in each package under `ros_bridge.hpp` in each respective package
Layout as follows:

* King Engine
    * get_location
    * set_destination
* Lidar
    * get_location
    * set_map
* Localization
    * an entry point (localization_main) passing a lambda to set location
* Path Plan
    * on_map_update
    * set_path
    * get_location
* Traversal
    * get_location
    * get_path

