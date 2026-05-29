#!/bin/bash
echo "🧹 Cleaning up previous session..."

# Kill ROS processes
pkill -f "ros2 bag record" 2>/dev/null
pkill -f "zed_camera" 2>/dev/null
pkill -f "zed_node" 2>/dev/null
pkill -f "component_container" 2>/dev/null
pkill -f "record_odometry" 2>/dev/null
pkill -f "capture_images" 2>/dev/null
pkill -f "ZED_Media_Server" 2>/dev/null
sleep 2

# Clean ZED shared memory / semaphores
rm -f /tmp/zed_* 2>/dev/null
rm -f /dev/shm/zed_* 2>/dev/null
rm -f /dev/shm/SL_* 2>/dev/null
rm -f /tmp/SL_* 2>/dev/null

# THE FIX: restart nvargus-daemon to release the stale argus_socket + camera session
echo "🔄 Restarting nvargus-daemon (releases stale camera session)..."
sudo systemctl restart nvargus-daemon

# Wait for daemon to fully reinitialize before ZED tries to connect
echo "⏳ Waiting for Argus to reinitialize..."
sleep 4

# Verify daemon is back up before proceeding
if ! systemctl is-active --quiet nvargus-daemon; then
    echo "❌ nvargus-daemon failed to restart! Aborting."
    exit 1
fi

echo "✅ Cleanup done. Launching..."
ros2 launch zed_odometry zed_record_odom.launch.py



























































































#!/bin/bash

#echo "🧹 Cleaning up previous session..."

# Kill any leftover ros2 bag record processes
#pkill -f "ros2 bag record" 2>/dev/null
#sleep 1

# Kill any leftover ZED ROS nodes from previous launch
#pkill -f "zed_camera" 2>/dev/null
#pkill -f "zed_node" 2>/dev/null
#pkill -f "component_container" 2>/dev/null
#pkill -f "record_odometry" 2>/dev/null
#pkill -f "capture_images" 2>/dev/null
#sleep 2

# Kill ZED_Media_Server if it's running as your user (won't work if root)
#pkill -f "ZED_Media_Server" 2>/dev/null
#sleep 1

#echo "✅ Cleanup done. Launching..."
#ros2 launch zed_odometry zed_record_odom.launch.py
