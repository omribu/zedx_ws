#!/bin/bash
echo "Starting experiment..."
ros2 service call /capture_start_image std_srvs/srv/Trigger &
ros2 service call /start_recording std_srvs/srv/Trigger
echo "Experiment started!"
