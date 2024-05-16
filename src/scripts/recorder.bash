#!/bin/bash

# Specify the topics you want to record
TOPICS="/cloud_all_fields_fullframe /filtered_imu /filtered_cloud /ImageRight /ImageLeft /ImageCenter"

# Record all specified topics, split into seperate files every 300 seconds, compress files
ros2 bag record $TOPICS -d 60 --compression-mode file --compression-format zstd
