#!/usr/bin/env python3

from email.mime import image
import os
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import cv2
import sqlite3

# REPLACE PATH AND NAME
BAG_PATH = "/home/volcani//workspaces/zedx_ws/zed_odom_recordings/17_02_2026/"
ExpName="CamPos3"
ExpNo="zed_odom_20260217_151231"   

class Analyzer:
    def __init__(self):
        # Storage for odometry data
        self.timestamps = []
        self.x_positions = []
        self.y_positions = []
        self.z_positions = []

        self.start_pixel = []
        self.stop_pixel = []

        self.r_prime_start = []
        self.r_prime_stop = []


    def read_bag(self, bag_path):
        """Read rosbag and extract odometry data"""
        print(f"Reading bag from: {bag_path}")
        
        # Find the .db3 file
        db_file = None
        for file in os.listdir(bag_path):
            if file.endswith('.db3'):
                db_file = os.path.join(bag_path, file)
                break
        
        if not db_file:
            print(f"No .db3 file found in {bag_path}")
            return False

        print(f"Reading database: {db_file}")

        try:
            conn = sqlite3.connect(db_file)
            cursor = conn.cursor()
            
            # Get topics
            cursor.execute("SELECT id, name, type FROM topics")
            topics = {row[0]: {'name': row[1], 'type': row[2]} for row in cursor.fetchall()}
            
            # Find odometry topic
            odom_topic_id = None
            for topic_id, topic_info in topics.items():
                if 'odom' in topic_info['name'].lower():
                    odom_topic_id = topic_id
                    print(f"Found odometry topic: {topic_info['name']}")
                    break
            
            if not odom_topic_id:
                print("No odometry topic found in bag")
                return False
            
            # Read messages from the odometry topic
            cursor.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
                (odom_topic_id,)
            )
            
            # Get message type
            msg_type = get_message(topics[odom_topic_id]['type'])
            
            # Process messages
            for timestamp, data in cursor.fetchall():
                msg = deserialize_message(data, msg_type)
                
                self.timestamps.append(timestamp * 1e-9)  # Convert to seconds
                self.x_positions.append(msg.pose.pose.position.x)
                self.y_positions.append(msg.pose.pose.position.y)
                self.z_positions.append(msg.pose.pose.position.z)
            
            conn.close()
            
            print(f"Loaded {len(self.x_positions)} odometry messages")
            return True
            
        except Exception as e:
            print(f"Error reading bag: {str(e)}")
            return False

  
    def plot_trajectory(self, x_start, y_start, x_stop, y_stop, error, save_path=None):
        """
        Plot the odometry trajectory with reference line and error circle.
        
        Parameters:
        - x_start, y_start: starting position in world frame
        - x_stop, y_stop: expected stopping position in world frame
        - error: total error (radius of error circle)
        - save_path: optional path to save the figure
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Plot odometry trajectory
        ax.plot(self.x_positions, self.y_positions, 
                'b-o', label='Odometry Trajectory', linewidth=2, markersize=3)
        
        # Plot reference straight line from start to stop
        ax.plot([x_start, x_stop], [y_start, y_stop], 
                'g--', label='Reference Path', linewidth=2)
        
        # Plot start and stop points
        ax.plot(x_start, y_start, 'go', markersize=12, label='Start Position', markeredgecolor='black', markeredgewidth=1.5)
        ax.annotate(f'({x_start:.3f}, {y_start:.3f}) m',
                    xy=(x_start, y_start),
                    xytext=(10, -15), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.4', facecolor='lightgreen', alpha=0.8),
                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
        ax.plot(x_stop, y_stop, 'ro', markersize=12, label='Expected End Position', markeredgecolor='black', markeredgewidth=1.5)
        
        # Plot actual odometry end point
        ax.plot(self.x_positions[-1], self.y_positions[-1], 
                'bs', markersize=12, label='Odometry End Position', markeredgecolor='black', markeredgewidth=1.5)
        
        # Plot error circle around expected stop position
        circle = plt.Circle((x_stop, y_stop), error, 
                            color='red', fill=False, linestyle='--', 
                            linewidth=2, label=f'Error Circle (r={error:.4f} m)')
        ax.add_patch(circle)
        
        # Add error annotation
        ax.annotate(f'Error: {error:.4f} m', 
                    xy=(self.x_positions[-1], self.y_positions[-1]),
                    xytext=(10, 10), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7),
                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title('Odometry Trajectory with Reference Path and Error', fontsize=14, fontweight='bold')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=10)
        
        # Add text box with statistics
        textstr = f'Distance traveled: {np.sqrt((x_stop - x_start)**2 + (y_stop - y_start)**2):.4f} m\n'
        textstr += f'Total error: {error:.4f} m\n'
        textstr += f'Error percentage: {(error / np.sqrt((x_stop - x_start)**2 + (y_stop - y_start)**2) * 100):.2f}%'
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', bbox=props)
        
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {save_path}")

        plt.close()

        # print("last x odometry position:", self.x_positions[-1])
        # print("last y odometry position:", self.y_positions[-1])


def main(args=None):
    rclpy.init(args=args)
    analyzer = Node('node')

    bag_path = BAG_PATH + ExpName + "/" + ExpNo + "/"
    
    odom_analyzer = Analyzer()

    
    if odom_analyzer.read_bag(bag_path):
    	print(f"odometry_x: {odom_analyzer.x_positions[-1]}")
    	print(f"odometry_y: {odom_analyzer.y_positions[-1]}")
    	"""
    	with open('/home/volcani/zedx_ws/zed_odom_recordings/ZEDDist.csv', 'a') as file:
    	    file.write(str(odom_analyzer.x_positions[-1])+";"+str(odom_analyzer.y_positions[-1])+";"+ExpName+";"+ExpNo+"\n")
    	"""
    	fig, ax = plt.subplots(figsize=(10, 8))

	ax.plot(odom_analyzer.x_positions, odom_analyzer.y_positions,'b-o', label='Odometry Trajectory', linewidth=2, markersize=3)

        plt.close()
        
    analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



