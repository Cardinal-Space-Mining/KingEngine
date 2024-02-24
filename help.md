 # Sourcing Ros2
 * `source /opt/ros/iron/setup.bash`
 # Building 
 * `colcon build`
 # Creating a package
 * `ros2 pkg create --build-type ament_cmake --license MIT <pkg name>`
 * Nodes can be prepopulated via  `ros2 pkg create --build-type ament_cmake --license MIT --node-name NODE_NAME <pkg name>`
 # Building
 * Update dependencies: `rosdep install -i --from-path src --rosdistro iron -y`
 * Build: `colcon build`
 * Source new nodes: `source ./install/setup.bash`
 # Running
 * `ros2 run <package name> <node name>`
 # Colcon Build is annoyed at using setuptools
 * `pip install setuptools==58.2.0`

 # VSCode autocomplete and include errors
 * Add `\opt\ros\iron\include\**` to vs code include path