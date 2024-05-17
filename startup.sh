colcon build --executor parallel
source ./install/setup.bash
ros2 launch king_engine lance.launch.py &
source ./src/scripts/recorder.bash &
python3 ./src/scripts/multibagger.py &
sudo motion  
echo "Go to 10.11.11.13:8080 For camera views."
echo "DONE!"