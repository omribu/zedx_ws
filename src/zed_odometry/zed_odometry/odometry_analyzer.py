#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3


class OdometryAnalyzer(Node):
    def __init__(self):
        super().__init__('odometry_analyzer')
        
        # Storage for odometry data
        self.timestamps = []
        self.x_positions = []
        self.y_positions = []
        self.z_positions = []
        
        # Ground truth configuration
        self.ground_truth_type = "straight_line"  # Options: "straight_line", "custom_points"
        self.ground_truth_params = {
            "start_point": [0.0, 0.0],
            "end_point": [10.0, 0.0],  # 10 meters along x-axis
        }
        
        self.get_logger().info("Odometry Analyzer initialized")
        

    def read_bag(self, bag_path):
        """Read rosbag and extract odometry data"""
        self.get_logger().info(f"Reading bag from: {bag_path}")
        
        # Find the .db3 file
        db_file = None
        for file in os.listdir(bag_path):
            if file.endswith('.db3'):
                db_file = os.path.join(bag_path, file)
                break
        
        if not db_file:
            self.get_logger().error(f"No .db3 file found in {bag_path}")
            return False
        
        self.get_logger().info(f"Reading database: {db_file}")
        
        try:
            # Connect to SQLite database
            conn = sqlite3.connect(db_file)
            cursor = conn.cursor()
            
            # Get topics
            cursor.execute("SELECT id, name, type FROM topics")
            topics = {row[0]: {'name': row[1], 'type': row[2]} for row in cursor.fetchall()}
            
            # Find the odometry topic
            odom_topic_id = None
            for topic_id, topic_info in topics.items():
                if 'odom' in topic_info['name'].lower():
                    odom_topic_id = topic_id
                    self.get_logger().info(f"Found odometry topic: {topic_info['name']}")
                    break
            
            if not odom_topic_id:
                self.get_logger().error("No odometry topic found in bag")
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
            
            self.get_logger().info(f"Loaded {len(self.x_positions)} odometry messages")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error reading bag: {str(e)}")
            return False


    def set_ground_truth_straight_line(self, start_point, end_point):
        """Set ground truth as a straight line"""
        self.ground_truth_type = "straight_line"
        self.ground_truth_params = {
            "start_point": start_point,
            "end_point": end_point
        }
        self.get_logger().info(f"Ground truth set: Line from {start_point} to {end_point}")


    def set_ground_truth_custom(self, waypoints):
        """Set ground truth as custom waypoints"""
        self.ground_truth_type = "custom_points"
        self.ground_truth_params = {"waypoints": waypoints}
        self.get_logger().info(f"Ground truth set: {len(waypoints)} waypoints")


    def generate_ground_truth(self, num_points=100):
        """Generate ground truth points based on configuration"""
        if self.ground_truth_type == "straight_line":
            start = np.array(self.ground_truth_params["start_point"])
            end = np.array(self.ground_truth_params["end_point"])
            
            gt_x = np.linspace(start[0], end[0], num_points)
            gt_y = np.linspace(start[1], end[1], num_points)
            
        elif self.ground_truth_type == "custom_points":
            waypoints = np.array(self.ground_truth_params["waypoints"])
            gt_x = waypoints[:, 0]
            gt_y = waypoints[:, 1]
            
        return gt_x, gt_y


    def calculate_errors(self):
        """Calculate errors between actual trajectory and ground truth"""
        if len(self.x_positions) == 0:
            self.get_logger().warn("No odometry data to analyze")
            return None
        
        actual_points = np.column_stack((self.x_positions, self.y_positions))
        
        if self.ground_truth_type == "straight_line":
            # Calculate perpendicular distance from line
            start = np.array(self.ground_truth_params["start_point"])
            end = np.array(self.ground_truth_params["end_point"])
            
            # Line equation: ax + by + c = 0
            line_vec = end - start
            line_length = np.linalg.norm(line_vec)
            
            if line_length == 0:
                self.get_logger().error("Start and end points are the same")
                return None
            
            # Perpendicular distance formula
            errors = []
            for point in actual_points:
                # Cross product method for distance to line
                dist = np.abs(np.cross(line_vec, start - point)) / line_length
                errors.append(dist)
            
            errors = np.array(errors)
            
            # Calculate along-track error (projection onto line)
            along_track_errors = []
            for point in actual_points:
                proj = np.dot(point - start, line_vec) / line_length
                along_track_errors.append(proj)
            along_track_errors = np.array(along_track_errors)
            
        elif self.ground_truth_type == "custom_points":
            # Calculate nearest point errors
            gt_points = np.array(self.ground_truth_params["waypoints"])
            errors = []
            
            for point in actual_points:
                distances = np.linalg.norm(gt_points - point, axis=1)
                min_dist = np.min(distances)
                errors.append(min_dist)
            
            errors = np.array(errors)
            along_track_errors = None
        
        # Calculate statistics
        results = {
            "errors": errors,
            "mean_error": np.mean(errors),
            "std_error": np.std(errors),
            "max_error": np.max(errors),
            "rmse": np.sqrt(np.mean(errors**2)),
            "along_track_errors": along_track_errors
        }
        
        return results


    def plot_trajectory(self, save_path=None):
        """Plot the trajectory with ground truth"""
        if len(self.x_positions) == 0:
            self.get_logger().warn("No data to plot")
            return
        
        # Create figure
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        
        # Plot 1: 2D Trajectory comparison
        ax1 = axes[0, 0]
        gt_x, gt_y = self.generate_ground_truth()
        
        ax1.plot(self.x_positions, self.y_positions, 'b-', linewidth=2, label='Actual Trajectory', alpha=0.7)
        ax1.plot(gt_x, gt_y, 'r--', linewidth=2, label='Ground Truth', alpha=0.7)
        ax1.plot(self.x_positions[0], self.y_positions[0], 'go', markersize=10, label='Start')
        ax1.plot(self.x_positions[-1], self.y_positions[-1], 'ro', markersize=10, label='End')
        
        ax1.set_xlabel('X Position (m)', fontsize=12)
        ax1.set_ylabel('Y Position (m)', fontsize=12)
        ax1.set_title('2D Trajectory Comparison', fontsize=14, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Plot 2: Error over time
        ax2 = axes[0, 1]
        error_results = self.calculate_errors()
        
        if error_results:
            time_relative = np.array(self.timestamps) - self.timestamps[0]
            
            ax2.plot(time_relative, error_results['errors'], 'b-', linewidth=1.5)
            ax2.axhline(y=error_results['mean_error'], color='r', linestyle='--', 
                       label=f'Mean: {error_results["mean_error"]:.3f} m')
            ax2.fill_between(time_relative, 
                            error_results['mean_error'] - error_results['std_error'],
                            error_results['mean_error'] + error_results['std_error'],
                            alpha=0.3, color='red', label=f'±1σ: {error_results["std_error"]:.3f} m')
            
            ax2.set_xlabel('Time (s)', fontsize=12)
            ax2.set_ylabel('Cross-Track Error (m)', fontsize=12)
            ax2.set_title('Error vs Time', fontsize=14, fontweight='bold')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        
        # Plot 3: Error distribution
        ax3 = axes[1, 0]
        if error_results:
            ax3.hist(error_results['errors'], bins=30, color='blue', alpha=0.7, edgecolor='black')
            ax3.axvline(error_results['mean_error'], color='r', linestyle='--', 
                       linewidth=2, label=f'Mean: {error_results["mean_error"]:.3f} m')
            ax3.axvline(error_results['mean_error'] + error_results['std_error'], 
                       color='orange', linestyle='--', linewidth=1.5, label=f'+1σ')
            ax3.axvline(error_results['mean_error'] - error_results['std_error'], 
                       color='orange', linestyle='--', linewidth=1.5, label=f'-1σ')
            
            ax3.set_xlabel('Error (m)', fontsize=12)
            ax3.set_ylabel('Frequency', fontsize=12)
            ax3.set_title('Error Distribution', fontsize=14, fontweight='bold')
            ax3.legend()
            ax3.grid(True, alpha=0.3, axis='y')
        
        # Plot 4: Statistics summary
        ax4 = axes[1, 1]
        ax4.axis('off')
        
        if error_results:
            stats_text = f"""
            TRAJECTORY STATISTICS
            ═══════════════════════════════
            
            Total Points:     {len(self.x_positions)}
            Duration:         {self.timestamps[-1] - self.timestamps[0]:.2f} s
            
            Distance Traveled: {self._calculate_path_length():.3f} m
            
            ERROR METRICS
            ═══════════════════════════════
            Mean Error:       {error_results['mean_error']:.4f} m
            Std Deviation:    {error_results['std_error']:.4f} m
            Max Error:        {error_results['max_error']:.4f} m
            RMSE:            {error_results['rmse']:.4f} m
            
            Ground Truth Type: {self.ground_truth_type.replace('_', ' ').title()}
            """
            
            ax4.text(0.1, 0.5, stats_text, fontsize=11, family='monospace',
                    verticalalignment='center', bbox=dict(boxstyle='round', 
                    facecolor='wheat', alpha=0.3))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            self.get_logger().info(f"Plot saved to: {save_path}")
        
        plt.show()


    def _calculate_path_length(self):
        """Calculate total path length"""
        if len(self.x_positions) < 2:
            return 0.0
        
        points = np.column_stack((self.x_positions, self.y_positions))
        diffs = np.diff(points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return np.sum(distances)


    def print_statistics(self):
        """Print detailed statistics"""
        error_results = self.calculate_errors()
        
        if not error_results:
            return
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("TRAJECTORY ANALYSIS RESULTS")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Total odometry points: {len(self.x_positions)}")
        self.get_logger().info(f"Duration: {self.timestamps[-1] - self.timestamps[0]:.2f} seconds")
        self.get_logger().info(f"Path length: {self._calculate_path_length():.3f} meters")
        self.get_logger().info("")
        self.get_logger().info("ERROR STATISTICS:")
        self.get_logger().info(f"  Mean error:       {error_results['mean_error']:.4f} m")
        self.get_logger().info(f"  Std deviation:    {error_results['std_error']:.4f} m")
        self.get_logger().info(f"  Max error:        {error_results['max_error']:.4f} m")
        self.get_logger().info(f"  RMSE:            {error_results['rmse']:.4f} m")
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    analyzer = Node('temp_node')
    
    # Odometry bag from recording ZEDx camera
    bag_path = "/home/volcani/zedx_ws/zed_odom_recordings/zed_odom_0.5zigzag"
    
    odom_analyzer = OdometryAnalyzer()
    
    # CUSTOMIZE ground truth #
    # Straight line 
    odom_analyzer.set_ground_truth_straight_line(
        start_point=[0.0, 0.0],
        end_point=[10.0, 0.0]
    )
    
    # Custom waypoints 
    # waypoints = [
    #     [0.0, 0.0],
    #     [2.0, 0.5],
    #     [4.0, 0.8],
    #     [6.0, 0.6],
    #     [8.0, 0.2],
    #     [10.0, 0.0]
    # ]
    # odom_analyzer.set_ground_truth_custom(waypoints)
    
    if odom_analyzer.read_bag(bag_path):
        # Print statistics
        odom_analyzer.print_statistics()
        
        # Plot results
        save_path = os.path.join(os.path.dirname(bag_path), "trajectory_analysis.png")
        odom_analyzer.plot_trajectory(save_path=save_path)
    
    analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()