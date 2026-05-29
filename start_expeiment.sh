#!/bin/bash
echo "Starting experiment..."

RESULT=$(ros2 service call /start_recording std_srvs/srv/Trigger 2>&1)
echo "$RESULT"

if echo "$RESULT" | grep -q "success=True"; then
    echo "✅ Odometry recording confirmed started"
    ros2 service call /capture_start_image std_srvs/srv/Trigger
    echo "Experiment started!"
else
    echo "❌ Recording FAILED — odom topic not live. Abort or retry."
    exit 1
fi
