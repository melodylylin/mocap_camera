#!/usr/bin/bash
ros2 run camera_calibration cameracalibrator \
--pattern charuco \
--size 6x8 \
--charuco_marker_size 0.105 \
--square 0.14 \
--aruco_dict 4x4_250 \
--ros-args -r image:=/camera_0/image -p camera:=/camera_0