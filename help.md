 # Sourcing ROS2
 * Run `source /opt/ros/<distro name>/setup.bash`
 	- ex. for humble: `source /opt/ros/humble/setup.bash`
 
 # Creating a package
 * To create a new package: `ros2 pkg create --build-type ament_cmake --license MIT <pkg name>`
 * Nodes can be prepopulated via  `ros2 pkg create --build-type ament_cmake --license MIT --node-name NODE_NAME <pkg name>`

 # Building
 * Update dependencies: `rosdep install -i --from-path src --rosdistro <distro name> -y`
 	- ex. for humble: `rosdep install -i --from-path src --rosdistro humble -y`
 * Build: `colcon build` (add `--executor parallel` for multiple cores, `--event-handlers console_direct+` to view live cmake output, `--packages-select`/`--packages-skip` to build specific packages only, or skip specific packages only, `--cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON` to make a compilation database)
	- ex. `colcon build --executor parallel --event-handlers console_direct+`
	- ex. `colcon build --executor parallel --event-handlers console_direct+ --packages-skip sick_scan_xd` because sick_scan_xd takes forever :/
 * Source new nodes: `source ./install/setup.bash`
 
 # Running
 1. Probably build the project
 2. Source the installation files: `source ./install/setup.sh`
 3. Run `ros2 run <package name> <node name>` (launchfiles to come)
 
 # Colcon Build is annoyed at using setuptools
 * `pip install setuptools==58.2.0`
 
 # VSCode autocomplete and include errors
 * Add `\opt\ros\<distro name>\include\**` to vs code include path
	- ex. for humble: `\opt\ros\humble\include\**`
