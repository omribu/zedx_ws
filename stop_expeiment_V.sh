#!/bin/bash
echo "Stopping experiment..."
ros2 service call /capture_stop_image std_srvs/srv/Trigger &
ros2 service call /stop_recording std_srvs/srv/Trigger
echo "Experiment stopped!"
