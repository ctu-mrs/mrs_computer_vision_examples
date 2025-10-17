#!/bin/bash

ros2 run tf2_ros static_transform_publisher \
  --x 0 \
  --y 0 \
  --z 0.1 \
  --roll -1.57 \
  --pitch 0 \
  --yaw -1.57 \
  --frame-id $UAV_NAME/fcu \
  --child-frame-id $UAV_NAME/bluefox_front_optical
