#!/bin/bash

# Specify the topics you want to record
TOPICS="/cloud_all_fields_fullframe /filtered_imu /filtered_cloud /ImageRight /ImageLeft /ImageCenter"

# Specify the filename for the bag file
BAG_FILE="my_bagfile"

# Record messages
ros2 bag record -o $BAG_FILE $TOPICS
