#!/usr/bin/bash
ros2 run camera_calibration cameracalibrator \
-c camera_0 \
-s 6x9 \
-q 0.02315 \
--ros-args -r image:=/camera_0/image -p camera:=/camera_0