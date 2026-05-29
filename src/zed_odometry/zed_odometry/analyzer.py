#!/usr/bin/env python3

from email.mime import image
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import cv2
import sqlite3

# This code analyzes expirement data - odometry data from a rosbag and images of the inital position and last position
# first 3 images pop up - by clicking first the pixel of the origin coordinate frame, than on the x axis direction 
# and lastly on the target point which is the projection of the zed camera on the plane.
# we do the same for the last image which pops up 3 images 
# after this the code calculates the world coordinates relative to the origin point and calculates 
# the error between the last odometry from the zed camera (x, y) and the calculated world coordinates from the last image.
# That way we can evaluate the odometry accuracy of the zed camera when traveling a certain distance in the environment of the 
# experiment.


# REPLACE PATH for specific experiment
NAME = "10b"
EXP_TYPE = "1.0m_s/WITH_MARKERS" 
DATE = "24_5_26"
BAG_PATH = "/home/volcani/workspaces/zedx_ws/zed_odom_recordings/" + DATE + "/" + EXP_TYPE + "/" + NAME + "/"

DISTANCE = -5.0  # meters — positive = forward, negative = backward

H = 0.725 # Camera height in meters from ground (along Z axis)

# !!!!   CALIBRATE THIS IF BOTH ZED AND REALSENSE CAMERA ARE PHYICALY MOVED AND BEFORE EACH RUN OF THE EXPERIMENT  !!!!!
PIXEL_QUERY = [520, 394]  # last camera setup


class PixelValueExtractor:

    @staticmethod
    def _get_screen_size():
        try:
            import tkinter as tk
            root = tk.Tk()
            root.withdraw()
            w, h = root.winfo_screenwidth(), root.winfo_screenheight()
            root.destroy()
            return w, h
        except Exception:
            return 1920, 1080

    @staticmethod
    def _scale_to_screen(image):
        screen_w, screen_h = PixelValueExtractor._get_screen_size()
        img_h, img_w = image.shape[:2]
        scale = min(screen_w / img_w, screen_h / img_h, 1.0)
        if scale < 1.0:
            new_w, new_h = int(img_w * scale), int(img_h * scale)
            return cv2.resize(image, (new_w, new_h)), scale
        return image, 1.0

    @staticmethod
    def get_pixel_value(image, window_name="Image"):
        if image is None:
            raise RuntimeError("Image is None")

        display_image, scale = PixelValueExtractor._scale_to_screen(image)
        clicked = {"done": False, "data": None}

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                orig_x = int(x / scale)
                orig_y = int(y / scale)
                clicked["data"] = (orig_x, orig_y, image[orig_y, orig_x].tolist())
                clicked["done"] = True

        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(window_name, display_image)
        cv2.setMouseCallback(window_name, mouse_callback)

        while not clicked["done"]:
            if cv2.waitKey(20) & 0xFF == 27:  # ESC cancels
                break

        cv2.destroyWindow(window_name)
        return clicked["data"]

    @staticmethod
    def get_n_pixel_values(image, n=2, window_name="Image"):
        if image is None:
            raise RuntimeError("Image is None")

        display_image, scale = PixelValueExtractor._scale_to_screen(image)
        clicks = []

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                orig_x = int(x / scale)
                orig_y = int(y / scale)
                clicks.append((orig_x, orig_y, image[orig_y, orig_x].tolist()))

        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(window_name, display_image)
        cv2.setMouseCallback(window_name, mouse_callback)

        while len(clicks) < n:
            if cv2.waitKey(20) & 0xFF == 27:  # ESC cancels
                break

        cv2.destroyWindow(window_name)
        return clicks


class Analyzer:
    def __init__(self):
        # Storage for odometry data
        self.timestamps = []
        self.x_positions = []
        self.y_positions = []
        self.z_positions = []

        self.bag_filename = ""

        self.start_pixel = []
        self.stop_pixel = []

        self.r_prime_start = []
        self.r_prime_stop = []

        self.K = np.array([
            [636.1937866210938, 0.0, 654.04248046875],
            [0.0, 635.5068969726562, 359.96063232421875],
            [0.0, 0.0, 1.0]
        ])

        self.DistCoeffs = np.array([-0.05570114776492119, 0.07012512534856796, -5.900441828998737e-05, 0.0006998045719228685, -0.0226144976913929])

        self.H = H    # Camera height in meters

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

        self.bag_filename = os.path.basename(db_file)
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

    def extract_pixel_values(self):

        start_path = BAG_PATH + "/start.png"
        stop_path  = BAG_PATH + "/stop.png"

        image_start = cv2.imread(start_path)
        image_stop  = cv2.imread(stop_path)

        if image_start is None:
            raise RuntimeError(f"Failed to load {start_path}")
        if image_stop is None:
            raise RuntimeError(f"Failed to load {stop_path}")

        print("➡ Click START point (2 clicks needed)")
        self.start_pixel = PixelValueExtractor.get_n_pixel_values(image_start, n=2, window_name="START IMAGE")

        print("➡ Click STOP point (2 clicks needed)")
        self.stop_pixel = PixelValueExtractor.get_n_pixel_values(image_stop, n=2, window_name="STOP IMAGE")

        print("start pixel :",self.start_pixel)
        print("stop pixel :",self.stop_pixel)
        return self.start_pixel, self.stop_pixel

    def pixel_to_world(self, pixel_origin, pixel_x_direction, K, H):
        """Convert pixel coordinates to world coordinates assuming flat ground plane."""
        
        def pixel_to_ground(pixel_data, K, H, distortion_coeffs=None):
            """Helper function to convert pixel to ground plane position."""
            u, v = pixel_data[0], pixel_data[1]
            
            # Undistort pixel if distortion coefficients are provided
            if distortion_coeffs is not None:
                pixel_undistorted = cv2.undistortPoints(
                    np.array([[u, v]], dtype=np.float32),
                    K,
                    distortion_coeffs,
                    P=K
                )[0][0]
                u_undist, v_undist = pixel_undistorted[0], pixel_undistorted[1]
            else:
                u_undist, v_undist = u, v

            fx = K[0, 0]
            fy = K[1, 1]
            cx = K[0, 2]
            cy = K[1, 2]
            
            # Convert to normalized image coordinates
            x_norm = (u_undist - cx) / fx
            y_norm = (v_undist - cy) / fy
            
            # Project to ground plane (z=0) from camera at height H
            # For camera looking down, the ground point is:
            x_ground = H * x_norm
            y_ground = -H * y_norm
            
            return np.array([x_ground, y_ground])

        P_origin = pixel_to_ground(pixel_origin, K, H, self.DistCoeffs)
        P_x_axis = pixel_to_ground(pixel_x_direction, K, H, self.DistCoeffs)
        P_query = pixel_to_ground(PIXEL_QUERY, K, H, self.DistCoeffs)

        # Translate so origin is at (0, 0)
        P_x_axis_translated = P_x_axis - P_origin
        P_query_translated = P_query - P_origin

        # Compute rotation to align X-axis direction with [1, 0]
        x_axis_vec = P_x_axis_translated
        x_axis_length = np.linalg.norm(x_axis_vec)
        
        if x_axis_length < 1e-6:
            raise ValueError("Origin and X-axis pixels are too close together")
        
        x_axis_unit = x_axis_vec / x_axis_length
        
        cos_theta = x_axis_unit[0]
        sin_theta = x_axis_unit[1]
        
        R = np.array([
            [cos_theta, sin_theta],
            [-sin_theta, cos_theta]
        ])
        
        P_world = R @ P_query_translated
        
        print(f"Camera frame positions:")
        print(f"  Origin: {P_origin}")
        print(f"  X-axis: {P_x_axis}")
        print(f"  Query: {P_query}")
        print(f"World frame position: ({P_world[0]:.4f}, {P_world[1]:.4f}) m")
        
        return P_world[0], P_world[1]

    def error_calculation(self, x_start, y_start, x_stop_world, y_stop_world):
        """Calculate error between odometry and image-based world coordinates."""

        self.x_positions = [x + x_start for x in self.x_positions]
        self.y_positions = [y + y_start for y in self.y_positions]

        zed_last_odom_x = self.x_positions[-1]
        zed_last_odom_y = self.y_positions[-1]

        delta_x = zed_last_odom_x - x_stop_world
        delta_y = zed_last_odom_y - y_stop_world

        total_error = np.sqrt(delta_x**2 + delta_y**2)

        print(f"\nOdometry last position: x={zed_last_odom_x:.4f} m, y={zed_last_odom_y:.4f} m")
        print(f"Ground truth endpoint:  x={x_stop_world:.4f} m, y={y_stop_world:.4f} m")
        print(f"Error: Δx={delta_x:.4f} m, Δy={delta_y:.4f} m, Total Error={total_error:.4f} m")

        return total_error
    

    def plot_trajectory(self, x_start, y_start, x_stop_world, y_stop_world, error, save_path=None):
        """
        Plot the odometry trajectory with reference points and error circle.

        Parameters:
        - x_start, y_start: PIXEL_QUERY position in start frame (odometry origin)
        - x_stop_world, y_stop_world: measured stop position = PIXEL_QUERY in stop frame + DISTANCE offset
        - error: distance from (DISTANCE, 0) to measured stop (radius of error circle)
        - save_path: optional path to save the figure
        """
        fig, ax = plt.subplots(figsize=(10, 8))

        # Odometry trajectory
        ax.plot(self.x_positions, self.y_positions,
                'b-o', label='Odometry Trajectory', linewidth=2, markersize=3)

        # Reference line from odometry start to desired endpoint
        ax.plot([x_start, x_stop_world], [y_start, y_stop_world],
                'g--', label='Reference Path', linewidth=2)

        # World origin (0, 0) — where the user clicked in the start image
        ax.plot(0, 0, 'k*', markersize=16, label='World Origin (0, 0)',
                markeredgecolor='black', markeredgewidth=1.0)

        ax.plot(DISTANCE, 0, 'y*', markersize=10, label=f'Target ({DISTANCE}, 0)',
                markeredgecolor='goldenrod', markeredgewidth=1.0)
        
        # Odometry start = PIXEL_QUERY in start frame
        ax.plot(x_start, y_start, 'go', markersize=12, label=f'Ground Truth Start ({x_start:.3f}, {y_start:.3f})',
                markeredgecolor='black', markeredgewidth=1.5)

        # Ground truth endpoint (PIXEL_QUERY in stop image, offset by DISTANCE)
        ax.plot(x_stop_world, y_stop_world, 'ro', markersize=12,
                label=f'Ground Truth Endpoint ({x_stop_world:.3f}, {y_stop_world:.3f})',
                markeredgecolor='black', markeredgewidth=1.5)

        # Odometry end point
        x_last = self.x_positions[-1]
        y_last = self.y_positions[-1]
        ax.plot(x_last, y_last, 'bs', markersize=12,
                label=f'Odometry End ({x_last:.3f}, {y_last:.3f})',
                markeredgecolor='black', markeredgewidth=1.5)

        circle = plt.Circle((x_stop_world, y_stop_world), error,
                            color='red', fill=False, linestyle='--',
                            linewidth=2, label=f'Error Circle (r={error:.4f} m)')
        ax.add_patch(circle)

        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title('Odometry Trajectory with Reference Path and Error', fontsize=14, fontweight='bold')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=10)

        textstr = f'Known distance: {abs(DISTANCE):.4f} m\n'
        textstr += f'Reference error: {error:.4f} m\n'
        textstr += f'Error percentage: {(error / abs(DISTANCE) * 100):.2f}%'
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', bbox=props)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {save_path}")

        plt.close()


CSV_PATH = "/home/volcani/workspaces/zedx_ws/zed_odom_recordings/results.csv"

CSV_HEADERS = [
    "date", "exp_type", "name", "bag_filename",
    "error",
    "odom_end_x", "odom_end_y",
    "gt_start_x", "gt_start_y",
    "gt_end_x", "gt_end_y",
]

def save_results_csv(name, date, exp_type, bag_filename,
                     error, odom_end_x, odom_end_y,
                     gt_start_x, gt_start_y,
                     gt_end_x, gt_end_y):
    file_exists = os.path.isfile(CSV_PATH) and os.path.getsize(CSV_PATH) > 0
    with open(CSV_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(CSV_HEADERS)
        writer.writerow([
            date, exp_type, name, bag_filename,
            round(error, 6),
            round(odom_end_x, 6), round(odom_end_y, 6),
            round(gt_start_x, 6), round(gt_start_y, 6),
            round(gt_end_x, 6), round(gt_end_y, 6),
        ])
    print(f"Results saved to: {CSV_PATH}")


def main(args=None):
    rclpy.init(args=args)
    analyzer = Node('node')

    bag_path = BAG_PATH
    
    odom_analyzer = Analyzer()

    
    if odom_analyzer.read_bag(bag_path):
        start_pixels, stop_pixels = odom_analyzer.extract_pixel_values()

        print("\n=== Processing START image ===")
        x_start, y_start = odom_analyzer.pixel_to_world(
            start_pixels[0],    
            start_pixels[1],   
            odom_analyzer.K,
            odom_analyzer.H
        )

        print(f"PIXEL_QUERY start position (odometry origin): x={x_start:.4f} m, y={y_start:.4f} m")
        print("\n=== Processing STOP image ===")

        x_stop_local, y_stop_local = odom_analyzer.pixel_to_world(
            stop_pixels[0],  
            stop_pixels[1],   
            odom_analyzer.K,
            odom_analyzer.H
        )
  
        x_stop_world = x_stop_local + DISTANCE
        y_stop_world = y_stop_local
        print(f"Measured stop world coordinates: x={x_stop_world:.4f} m, y={y_stop_world:.4f} m")

        error = odom_analyzer.error_calculation(x_start, y_start, x_stop_world, y_stop_world)

        save_results_csv(
            name=NAME,
            date=DATE,
            exp_type=EXP_TYPE,
            bag_filename=odom_analyzer.bag_filename,
            error=error,
            odom_end_x=odom_analyzer.x_positions[-1],
            odom_end_y=odom_analyzer.y_positions[-1],
            gt_start_x=x_start,
            gt_start_y=y_start,
            gt_end_x=x_stop_world,
            gt_end_y=y_stop_world,
        )

        print(f"odometry_x: {odom_analyzer.x_positions[0]}")
        print(f"odometry_y: {odom_analyzer.y_positions[0]}")

        save_path = os.path.join(os.path.dirname(bag_path), NAME + ".png")
        odom_analyzer.plot_trajectory(x_start, y_start, x_stop_world, y_stop_world, error, save_path)
    
    
    analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()










































































































# #!/usr/bin/env python3

# from email.mime import image
# import csv
# import os
# import numpy as np
# import matplotlib.pyplot as plt
# import rclpy
# from rclpy.node import Node
# from rclpy.serialization import deserialize_message
# from rosidl_runtime_py.utilities import get_message
# import cv2
# import sqlite3

# # This code analyzes expirement data - odometry data from a rosbag and images of the inital position and last position
# # first 3 images pop up - by clicking first the pixel of the origin coordinate frame, than on the x axis direction 
# # and lastly on the target point which is the projection of the zed camera on the plane.
# # we do the same for the last image which pops up 3 images 
# # after this the code calculates the world coordinates relative to the origin point and calculates 
# # the error between the last odometry from the zed camera (x, y) and the calculated world coordinates from the last image.
# # That way we can evaluate the odometry accuracy of the zed camera when traveling a certain distance in the environment of the 
# # experiment.


# # REPLACE PATH for specific experiment
# NAME = "1"
# EXP_TYPE = "1.0m_s/10meter" 
# DATE = "7_5_26"
# BAG_PATH = "/home/volcani/workspaces/zedx_ws/zed_odom_recordings/" + DATE + "/" + EXP_TYPE + "/" + NAME + "/"

# DISTANCE = 5.0  # meters — positive = forward, negative = backward

# H = 0.725 # Camera height in meters from ground (along Z axis)

# # !!!!   CALIBRATE THIS IF BOTH ZED AND REALSENSE CAMERA ARE PHYICALY MOVED AND BEFORE EACH RUN OF THE EXPERIMENT  !!!!!
# PIXEL_QUERY = [520, 394]  # last camera setup


# class PixelValueExtractor:

#     @staticmethod
#     def get_pixel_value(image, window_name="Image"):
#         if image is None:
#             raise RuntimeError("Image is None")

#         clicked = {"done": False, "data": None}

#         def mouse_callback(event, x, y, flags, param):
#             if event == cv2.EVENT_LBUTTONDOWN:
#                 clicked["data"] = (x, y, image[y, x].tolist())
#                 clicked["done"] = True
#                 # print(f"Clicked ({x}, {y}) value={image[y, x]}")

#         cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
#         cv2.imshow(window_name, image)
#         cv2.setMouseCallback(window_name, mouse_callback)

#         while not clicked["done"]:
#             if cv2.waitKey(20) & 0xFF == 27:  # ESC cancels
#                 break

#         cv2.destroyWindow(window_name)
#         return clicked["data"]


# class Analyzer:
#     def __init__(self):
#         # Storage for odometry data
#         self.timestamps = []
#         self.x_positions = []
#         self.y_positions = []
#         self.z_positions = []

#         self.bag_filename = ""

#         self.start_pixel = []
#         self.stop_pixel = []

#         self.r_prime_start = []
#         self.r_prime_stop = []

#         self.K = np.array([
#             [636.1937866210938, 0.0, 654.04248046875],
#             [0.0, 635.5068969726562, 359.96063232421875],
#             [0.0, 0.0, 1.0]
#         ])

#         self.DistCoeffs = np.array([-0.05570114776492119, 0.07012512534856796, -5.900441828998737e-05, 0.0006998045719228685, -0.0226144976913929])

#         self.H = H    # Camera height in meters

#     def read_bag(self, bag_path):
#         """Read rosbag and extract odometry data"""
#         print(f"Reading bag from: {bag_path}")
        
#         # Find the .db3 file
#         db_file = None
#         for file in os.listdir(bag_path):
#             if file.endswith('.db3'):
#                 db_file = os.path.join(bag_path, file)
#                 break
        
#         if not db_file:
#             print(f"No .db3 file found in {bag_path}")
#             return False

#         self.bag_filename = os.path.basename(db_file)
#         print(f"Reading database: {db_file}")

#         try:
#             conn = sqlite3.connect(db_file)
#             cursor = conn.cursor()
            
#             # Get topics
#             cursor.execute("SELECT id, name, type FROM topics")
#             topics = {row[0]: {'name': row[1], 'type': row[2]} for row in cursor.fetchall()}
            
#             # Find odometry topic
#             odom_topic_id = None
#             for topic_id, topic_info in topics.items():
#                 if 'odom' in topic_info['name'].lower():
#                     odom_topic_id = topic_id
#                     print(f"Found odometry topic: {topic_info['name']}")
#                     break
            
#             if not odom_topic_id:
#                 print("No odometry topic found in bag")
#                 return False
            
#             # Read messages from the odometry topic
#             cursor.execute(
#                 "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
#                 (odom_topic_id,)
#             )
            
#             # Get message type
#             msg_type = get_message(topics[odom_topic_id]['type'])
            
#             # Process messages
#             for timestamp, data in cursor.fetchall():
#                 msg = deserialize_message(data, msg_type)
                
#                 self.timestamps.append(timestamp * 1e-9)  # Convert to seconds
#                 self.x_positions.append(msg.pose.pose.position.x)
#                 self.y_positions.append(msg.pose.pose.position.y)
#                 self.z_positions.append(msg.pose.pose.position.z)
            
#             conn.close()
            
#             print(f"Loaded {len(self.x_positions)} odometry messages")
#             return True
            
#         except Exception as e:
#             print(f"Error reading bag: {str(e)}")
#             return False

#     def extract_pixel_values(self):

#         start_path = BAG_PATH + "/start.png"
#         stop_path  = BAG_PATH + "/stop.png"

#         image_start = cv2.imread(start_path)
#         image_stop  = cv2.imread(stop_path)

#         if image_start is None:
#             raise RuntimeError(f"Failed to load {start_path}")
#         if image_stop is None:
#             raise RuntimeError(f"Failed to load {stop_path}")

#         print("➡ Click START point")
#         for i in range(2):
#             pixel = PixelValueExtractor.get_pixel_value(
#                 image_start, window_name="START IMAGE"
#             )
#             if pixel is not None:
#                 self.start_pixel.append(pixel)

#         print("➡ Click STOP point")
#         for i in range(2):
#             pixel = PixelValueExtractor.get_pixel_value(
#                 image_stop, window_name="STOP IMAGE"
#             )
#             if pixel is not None:
#                 self.stop_pixel.append(pixel)

#         print("start pixel :",self.start_pixel)
#         print("stop pixel :",self.stop_pixel)
#         return self.start_pixel, self.stop_pixel

#     def pixel_to_world(self, pixel_origin, pixel_x_direction, K, H):
#         """Convert pixel coordinates to world coordinates assuming flat ground plane."""
        
#         def pixel_to_ground(pixel_data, K, H, distortion_coeffs=None):
#             """Helper function to convert pixel to ground plane position."""
#             u, v = pixel_data[0], pixel_data[1]
            
#             # Undistort pixel if distortion coefficients are provided
#             if distortion_coeffs is not None:
#                 pixel_undistorted = cv2.undistortPoints(
#                     np.array([[u, v]], dtype=np.float32),
#                     K,
#                     distortion_coeffs,
#                     P=K
#                 )[0][0]
#                 u_undist, v_undist = pixel_undistorted[0], pixel_undistorted[1]
#             else:
#                 u_undist, v_undist = u, v
            
#             # Extract intrinsic parameters
#             fx = K[0, 0]
#             fy = K[1, 1]
#             cx = K[0, 2]
#             cy = K[1, 2]
            
#             # Convert to normalized image coordinates
#             x_norm = (u_undist - cx) / fx
#             y_norm = (v_undist - cy) / fy
            
#             # Project to ground plane (z=0) from camera at height H
#             # For camera looking down, the ground point is:
#             x_ground = H * x_norm
#             y_ground = -H * y_norm
            
#             return np.array([x_ground, y_ground])

#         P_origin = pixel_to_ground(pixel_origin, K, H, self.DistCoeffs)
#         P_x_axis = pixel_to_ground(pixel_x_direction, K, H, self.DistCoeffs)
#         P_query = pixel_to_ground(PIXEL_QUERY, K, H, self.DistCoeffs)

#         # Step 1: Translate so origin is at (0, 0)
#         P_x_axis_translated = P_x_axis - P_origin
#         P_query_translated = P_query - P_origin

#         # Step 2: Compute rotation to align X-axis direction with [1, 0]
#         x_axis_vec = P_x_axis_translated
#         x_axis_length = np.linalg.norm(x_axis_vec)
        
#         if x_axis_length < 1e-6:
#             raise ValueError("Origin and X-axis pixels are too close together")
        
#         # Normalize to get unit vector
#         x_axis_unit = x_axis_vec / x_axis_length
        
#         # Rotation matrix to align x_axis_unit with [1, 0]
#         cos_theta = x_axis_unit[0]
#         sin_theta = x_axis_unit[1]
        
#         # Rotation matrix (rotates by -theta to align with standard axes)
#         R = np.array([
#             [cos_theta, sin_theta],
#             [-sin_theta, cos_theta]
#         ])
        
#         # Step 3: Apply rotation to query point
#         P_world = R @ P_query_translated
        
#         print(f"Camera frame positions:")
#         print(f"  Origin: {P_origin}")
#         print(f"  X-axis: {P_x_axis}")
#         print(f"  Query: {P_query}")
#         print(f"World frame position: ({P_world[0]:.4f}, {P_world[1]:.4f}) m")
        
#         return P_world[0], P_world[1]

#     def error_calculation(self, x_start, y_start, x_stop_world, y_stop_world):
#         """Calculate error between odometry and image-based world coordinates."""

#         # Translate odometry to start at PIXEL_QUERY position in start frame
#         self.x_positions = [x + x_start for x in self.x_positions]
#         self.y_positions = [y + y_start for y in self.y_positions]

#         zed_last_odom_x = self.x_positions[-1]
#         zed_last_odom_y = self.y_positions[-1]

#         # Error = distance from odometry last point to ground truth endpoint
#         delta_x = zed_last_odom_x - x_stop_world
#         delta_y = zed_last_odom_y - y_stop_world

#         total_error = np.sqrt(delta_x**2 + delta_y**2)

#         print(f"\nOdometry last position: x={zed_last_odom_x:.4f} m, y={zed_last_odom_y:.4f} m")
#         print(f"Ground truth endpoint:  x={x_stop_world:.4f} m, y={y_stop_world:.4f} m")
#         print(f"Error: Δx={delta_x:.4f} m, Δy={delta_y:.4f} m, Total Error={total_error:.4f} m")

#         return total_error
    

#     def plot_trajectory(self, x_start, y_start, x_stop_world, y_stop_world, error, save_path=None):
#         """
#         Plot the odometry trajectory with reference points and error circle.

#         Parameters:
#         - x_start, y_start: PIXEL_QUERY position in start frame (odometry origin)
#         - x_stop_world, y_stop_world: measured stop position = PIXEL_QUERY in stop frame + DISTANCE offset
#         - error: distance from (DISTANCE, 0) to measured stop (radius of error circle)
#         - save_path: optional path to save the figure
#         """
#         fig, ax = plt.subplots(figsize=(10, 8))

#         # Odometry trajectory
#         ax.plot(self.x_positions, self.y_positions,
#                 'b-o', label='Odometry Trajectory', linewidth=2, markersize=3)

#         # Reference line from odometry start to desired endpoint
#         ax.plot([x_start, x_stop_world], [y_start, y_stop_world],
#                 'g--', label='Reference Path', linewidth=2)

#         # World origin (0, 0) — where the user clicked in the start image
#         ax.plot(0, 0, 'k*', markersize=16, label='World Origin (0, 0)',
#                 markeredgecolor='black', markeredgewidth=1.0)

#         ax.plot(DISTANCE, 0, 'y*', markersize=10, label=f'Target ({DISTANCE}, 0)',
#                 markeredgecolor='goldenrod', markeredgewidth=1.0)
        
#         # Odometry start = PIXEL_QUERY in start frame
#         ax.plot(x_start, y_start, 'go', markersize=12, label=f'Ground Truth Start ({x_start:.3f}, {y_start:.3f})',
#                 markeredgecolor='black', markeredgewidth=1.5)

#         # Ground truth endpoint (PIXEL_QUERY in stop image, offset by DISTANCE)
#         ax.plot(x_stop_world, y_stop_world, 'ro', markersize=12,
#                 label=f'Ground Truth Endpoint ({x_stop_world:.3f}, {y_stop_world:.3f})',
#                 markeredgecolor='black', markeredgewidth=1.5)

#         # Odometry end point
#         x_last = self.x_positions[-1]
#         y_last = self.y_positions[-1]
#         ax.plot(x_last, y_last, 'bs', markersize=12,
#                 label=f'Odometry End ({x_last:.3f}, {y_last:.3f})',
#                 markeredgecolor='black', markeredgewidth=1.5)

#         # Error circle centered at ground truth endpoint
#         circle = plt.Circle((x_stop_world, y_stop_world), error,
#                             color='red', fill=False, linestyle='--',
#                             linewidth=2, label=f'Error Circle (r={error:.4f} m)')
#         ax.add_patch(circle)

#         ax.set_xlabel('X Position (m)', fontsize=12)
#         ax.set_ylabel('Y Position (m)', fontsize=12)
#         ax.set_title('Odometry Trajectory with Reference Path and Error', fontsize=14, fontweight='bold')
#         ax.axis('equal')
#         ax.grid(True, alpha=0.3)
#         ax.legend(loc='best', fontsize=10)

#         textstr = f'Known distance: {abs(DISTANCE):.4f} m\n'
#         textstr += f'Reference error: {error:.4f} m\n'
#         textstr += f'Error percentage: {(error / abs(DISTANCE) * 100):.2f}%'
#         props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
#         ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10,
#                 verticalalignment='top', bbox=props)

#         plt.tight_layout()

#         if save_path:
#             plt.savefig(save_path, dpi=300, bbox_inches='tight')
#             print(f"Plot saved to: {save_path}")

#         plt.show()


# CSV_PATH = "/home/volcani/workspaces/zedx_ws/zed_odom_recordings/results.csv"

# CSV_HEADERS = [
#     "date", "exp_type", "name", "bag_filename",
#     "error",
#     "odom_end_x", "odom_end_y",
#     "gt_start_x", "gt_start_y",
#     "gt_end_x", "gt_end_y",
# ]

# def save_results_csv(name, date, exp_type, bag_filename,
#                      error, odom_end_x, odom_end_y,
#                      gt_start_x, gt_start_y,
#                      gt_end_x, gt_end_y):
#     file_exists = os.path.isfile(CSV_PATH) and os.path.getsize(CSV_PATH) > 0
#     with open(CSV_PATH, "a", newline="") as f:
#         writer = csv.writer(f)
#         if not file_exists:
#             writer.writerow(CSV_HEADERS)
#         writer.writerow([
#             date, exp_type, name, bag_filename,
#             round(error, 6),
#             round(odom_end_x, 6), round(odom_end_y, 6),
#             round(gt_start_x, 6), round(gt_start_y, 6),
#             round(gt_end_x, 6), round(gt_end_y, 6),
#         ])
#     print(f"Results saved to: {CSV_PATH}")


# def main(args=None):
#     rclpy.init(args=args)
#     analyzer = Node('node')

#     bag_path = BAG_PATH
    
#     odom_analyzer = Analyzer()

    
#     if odom_analyzer.read_bag(bag_path):
#         start_pixels, stop_pixels = odom_analyzer.extract_pixel_values()

#         print("\n=== Processing START image ===")
#         x_start, y_start = odom_analyzer.pixel_to_world(
#             start_pixels[0],    # origin pixel
#             start_pixels[1],    # x direction pixel
#             odom_analyzer.K,
#             odom_analyzer.H
#         )

#         print(f"PIXEL_QUERY start position (odometry origin): x={x_start:.4f} m, y={y_start:.4f} m")
#         print("\n=== Processing STOP image ===")

#         x_stop_local, y_stop_local = odom_analyzer.pixel_to_world(
#             stop_pixels[0],   # origin pixel (physically at DISTANCE, 0)
#             stop_pixels[1],   # x direction pixel
#             odom_analyzer.K,
#             odom_analyzer.H
#         )
#         # World position of PIXEL_QUERY at stop = offset from (DISTANCE, 0)
#         x_stop_world = x_stop_local + DISTANCE
#         y_stop_world = y_stop_local
#         print(f"Measured stop world coordinates: x={x_stop_world:.4f} m, y={y_stop_world:.4f} m")

#         error = odom_analyzer.error_calculation(x_start, y_start, x_stop_world, y_stop_world)

#         save_results_csv(
#             name=NAME,
#             date=DATE,
#             exp_type=EXP_TYPE,
#             bag_filename=odom_analyzer.bag_filename,
#             error=error,
#             odom_end_x=odom_analyzer.x_positions[-1],
#             odom_end_y=odom_analyzer.y_positions[-1],
#             gt_start_x=x_start,
#             gt_start_y=y_start,
#             gt_end_x=x_stop_world,
#             gt_end_y=y_stop_world,
#         )

#         print(f"odometry_x: {odom_analyzer.x_positions[0]}")
#         print(f"odometry_y: {odom_analyzer.y_positions[0]}")

#         save_path = os.path.join(os.path.dirname(bag_path), NAME + ".png")
#         odom_analyzer.plot_trajectory(x_start, y_start, x_stop_world, y_stop_world, error, save_path)
    
    
#     analyzer.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




















































































































# #!/usr/bin/env python3

# from email.mime import image
# import os
# import numpy as np
# import matplotlib.pyplot as plt
# import rclpy
# from rclpy.node import Node
# from rclpy.serialization import deserialize_message
# from rosidl_runtime_py.utilities import get_message
# import cv2
# import sqlite3

# # This code analyzes expirement data - odometry data from a rosbag and images of the inital position and last position
# # first 3 images pop up - by clicking first the pixel of the origin coordinate frame, than on the x axis direction 
# # and lastly on the target point which is the projection of the zed camera on the plane.
# # we do the same for the last image which pops up 3 images 
# # after this the code calculates the world coordinates relative to the origin point and calculates 
# # the error between the last odometry from the zed camera (x, y) and the calculated world coordinates from the last image.
# # That way we can evaluate the odometry accuracy of the zed camera when traveling a certain distance in the environment of the 
# # experiment.


# # REPLACE PATH for specific experiment
# NAME = "10b"
# EXP_TYPE = "1.0m_s/1m_s5mMarks/" 
# DATE = "24_5_26"
# BAG_PATH = "/home/volcani/workspaces/zedx_ws/zed_odom_recordings/" + DATE + "/" + EXP_TYPE + "/" + NAME + "/"

# DISTANCE = -5.0  # meters  (add the position of the last position info to the x_stop, y_stop) (5.0, 0.0) for 5 meters forward

# H = 0.725 # Camera height in meters from ground (along Z axis)

# # CALIBRATE THIS IF THE REALSENSE CAMERA IS PHYICALY MOVED!!!
# PIXEL_QUERY = [520, 394]  # last camera setup


# class PixelValueExtractor:

#     @staticmethod
#     def get_pixel_value(image, window_name="Image"):
#         if image is None:
#             raise RuntimeError("Image is None")

#         clicked = {"done": False, "data": None}

#         def mouse_callback(event, x, y, flags, param):
#             if event == cv2.EVENT_LBUTTONDOWN:
#                 clicked["data"] = (x, y, image[y, x].tolist())
#                 clicked["done"] = True
#                 # print(f"Clicked ({x}, {y}) value={image[y, x]}")

#         cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
#         cv2.imshow(window_name, image)
#         cv2.setMouseCallback(window_name, mouse_callback)

#         while not clicked["done"]:
#             if cv2.waitKey(20) & 0xFF == 27:  # ESC cancels
#                 break

#         cv2.destroyWindow(window_name)
#         return clicked["data"]


# class Analyzer:
#     def __init__(self):
#         # Storage for odometry data
#         self.timestamps = []
#         self.x_positions = []
#         self.y_positions = []
#         self.z_positions = []

#         self.start_pixel = []
#         self.stop_pixel = []

#         self.r_prime_start = []
#         self.r_prime_stop = []

#         self.K = np.array([
#             [636.1937866210938, 0.0, 654.04248046875],
#             [0.0, 635.5068969726562, 359.96063232421875],
#             [0.0, 0.0, 1.0]
#         ])

#         self.DistCoeffs = np.array([-0.05570114776492119, 0.07012512534856796, -5.900441828998737e-05, 0.0006998045719228685, -0.0226144976913929])

#         # MY CALIBRATION
#         # self.K_my = np.array([
#         #     [470.51290201, 0.0, 335.27920402],
#         #     [0.0, 476.33936417, 212.38990294],
#         #     [0.0, 0.0, 1.0]
#         # ])

#         # self.DistCoeffs_my = np.array([0.19677268, -0.40855946, -0.03181466, 0.0100384, 1.439907])

#         self.H = H    # Camera height in meters

#     def read_bag(self, bag_path):
#         """Read rosbag and extract odometry data"""
#         print(f"Reading bag from: {bag_path}")
        
#         # Find the .db3 file
#         db_file = None
#         for file in os.listdir(bag_path):
#             if file.endswith('.db3'):
#                 db_file = os.path.join(bag_path, file)
#                 break
        
#         if not db_file:
#             print(f"No .db3 file found in {bag_path}")
#             return False

#         print(f"Reading database: {db_file}")

#         try:
#             conn = sqlite3.connect(db_file)
#             cursor = conn.cursor()
            
#             # Get topics
#             cursor.execute("SELECT id, name, type FROM topics")
#             topics = {row[0]: {'name': row[1], 'type': row[2]} for row in cursor.fetchall()}
            
#             # Find odometry topic
#             odom_topic_id = None
#             for topic_id, topic_info in topics.items():
#                 if 'odom' in topic_info['name'].lower():
#                     odom_topic_id = topic_id
#                     print(f"Found odometry topic: {topic_info['name']}")
#                     break
            
#             if not odom_topic_id:
#                 print("No odometry topic found in bag")
#                 return False
            
#             # Read messages from the odometry topic
#             cursor.execute(
#                 "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
#                 (odom_topic_id,)
#             )
            
#             # Get message type
#             msg_type = get_message(topics[odom_topic_id]['type'])
            
#             # Process messages
#             for timestamp, data in cursor.fetchall():
#                 msg = deserialize_message(data, msg_type)
                
#                 self.timestamps.append(timestamp * 1e-9)  # Convert to seconds
#                 self.x_positions.append(msg.pose.pose.position.x)
#                 self.y_positions.append(msg.pose.pose.position.y)
#                 self.z_positions.append(msg.pose.pose.position.z)
            
#             conn.close()
            
#             print(f"Loaded {len(self.x_positions)} odometry messages")
#             return True
            
#         except Exception as e:
#             print(f"Error reading bag: {str(e)}")
#             return False

#     def extract_pixel_values(self):

#         start_path = BAG_PATH + "/start.png"
#         stop_path  = BAG_PATH + "/stop.png"

#         image_start = cv2.imread(start_path)
#         image_stop  = cv2.imread(stop_path)

#         if image_start is None:
#             raise RuntimeError(f"Failed to load {start_path}")
#         if image_stop is None:
#             raise RuntimeError(f"Failed to load {stop_path}")

#         print("➡ Click START point")
#         for i in range(2):
#             pixel = PixelValueExtractor.get_pixel_value(
#                 image_start, window_name="START IMAGE"
#             )
#             if pixel is not None:
#                 self.start_pixel.append(pixel)

#         print("➡ Click STOP point")
#         for i in range(2):
#             pixel = PixelValueExtractor.get_pixel_value(
#                 image_stop, window_name="STOP IMAGE"
#             )
#             if pixel is not None:
#                 self.stop_pixel.append(pixel)

#         print("start pixel :",self.start_pixel)
#         print("stop pixel :",self.stop_pixel)
#         return self.start_pixel, self.stop_pixel

#     def pixel_to_world(self, pixel_origin, pixel_x_direction, K, H):
#         """Convert pixel coordinates to world coordinates assuming flat ground plane."""
        
#         def pixel_to_ground(pixel_data, K, H, distortion_coeffs=None):
#             """Helper function to convert pixel to ground plane position."""
#             u, v = pixel_data[0], pixel_data[1]
            
#             # Undistort pixel if distortion coefficients are provided
#             if distortion_coeffs is not None:
#                 pixel_undistorted = cv2.undistortPoints(
#                     np.array([[u, v]], dtype=np.float32),
#                     K,
#                     distortion_coeffs,
#                     P=K
#                 )[0][0]
#                 u_undist, v_undist = pixel_undistorted[0], pixel_undistorted[1]
#             else:
#                 u_undist, v_undist = u, v
            
#             # Extract intrinsic parameters
#             fx = K[0, 0]
#             fy = K[1, 1]
#             cx = K[0, 2]
#             cy = K[1, 2]
            
#             # Convert to normalized image coordinates
#             x_norm = (u_undist - cx) / fx
#             y_norm = (v_undist - cy) / fy
            
#             # Project to ground plane (z=0) from camera at height H
#             # For camera looking down, the ground point is:
#             x_ground = H * x_norm
#             y_ground = -H * y_norm
            
#             return np.array([x_ground, y_ground])

#         P_origin = pixel_to_ground(pixel_origin, K, H, self.DistCoeffs)
#         P_x_axis = pixel_to_ground(pixel_x_direction, K, H, self.DistCoeffs)
#         P_query = pixel_to_ground(PIXEL_QUERY, K, H, self.DistCoeffs)

#         # Step 1: Translate so origin is at (0, 0)
#         P_x_axis_translated = P_x_axis - P_origin
#         P_query_translated = P_query - P_origin

#         # Step 2: Compute rotation to align X-axis direction with [1, 0]
#         x_axis_vec = P_x_axis_translated
#         x_axis_length = np.linalg.norm(x_axis_vec)
        
#         if x_axis_length < 1e-6:
#             raise ValueError("Origin and X-axis pixels are too close together")
        
#         # Normalize to get unit vector
#         x_axis_unit = x_axis_vec / x_axis_length
        
#         # Rotation matrix to align x_axis_unit with [1, 0]
#         cos_theta = x_axis_unit[0]
#         sin_theta = x_axis_unit[1]
        
#         # Rotation matrix (rotates by -theta to align with standard axes)
#         R = np.array([
#             [cos_theta, sin_theta],
#             [-sin_theta, cos_theta]
#         ])
        
#         # Step 3: Apply rotation to query point
#         P_world = R @ P_query_translated
        
#         print(f"Camera frame positions:")
#         print(f"  Origin: {P_origin}")
#         print(f"  X-axis: {P_x_axis}")
#         print(f"  Query: {P_query}")
#         print(f"World frame position: ({P_world[0]:.4f}, {P_world[1]:.4f}) m")
        
#         return P_world[0], P_world[1]

#     def error_calculation(self, x_start, y_start, x_stop, y_stop):
#         """Calculate error between odometry and image-based world coordinates."""
        
#         # Translate odometry to start at the world origin x_start, y_start (BOTH WORLD FRAME AND ZED ODOMETRY (X FORWARD AND Y TO THE LEFT))
#         self.x_positions = [x + x_start for x in self.x_positions]
#         self.y_positions = [y + y_start for y in self.y_positions]


#         # Odometry displacement
#         zed_last_odom_x = self.x_positions[-1] 
#         zed_last_odom_y = self.y_positions[-1] 

#         delta_x = zed_last_odom_x - x_stop
#         delta_y = zed_last_odom_y - y_stop

#         total_error = np.sqrt(delta_x**2 + delta_y**2)

#         print(f"\nOdometry position: x={zed_last_odom_x:.4f} m, y={zed_last_odom_y:.4f} m")
#         print(f"Image-based position: x={x_stop:.4f} m, y={y_stop:.4f} m")
#         print(f"Error: Δx={delta_x:.4f} m, Δy={delta_y:.4f} m, Total Error={total_error:.4f} m")

#         return total_error
    

#     def plot_trajectory(self, x_start, y_start, x_stop, y_stop, error, save_path=None):
#         """
#         Plot the odometry trajectory with reference line and error circle.
        
#         Parameters:
#         - x_start, y_start: starting position in world frame
#         - x_stop, y_stop: expected stopping position in world frame
#         - error: total error (radius of error circle)
#         - save_path: optional path to save the figure
#         """
#         fig, ax = plt.subplots(figsize=(10, 8))
        
#         # Plot odometry trajectory
#         ax.plot(self.x_positions, self.y_positions, 
#                 'b-o', label='Odometry Trajectory', linewidth=2, markersize=3)
        
#         # Plot reference straight line from start to stop
#         ax.plot([x_start, x_stop], [y_start, y_stop], 
#                 'g--', label='Reference Path', linewidth=2)
        
#         # Plot start and stop points
#         ax.plot(x_start, y_start, 'go', markersize=12, label='Start Position', markeredgecolor='black', markeredgewidth=1.5)
#         ax.plot(x_stop, y_stop, 'ro', markersize=12, label='Expected End Position', markeredgecolor='black', markeredgewidth=1.5)
        
#         # Plot actual odometry end point
#         ax.plot(self.x_positions[-1], self.y_positions[-1], 
#                 'bs', markersize=12, label='Odometry End Position', markeredgecolor='black', markeredgewidth=1.5)
        
#         # Plot error circle around expected stop position
#         circle = plt.Circle((x_stop, y_stop), error, 
#                             color='red', fill=False, linestyle='--', 
#                             linewidth=2, label=f'Error Circle (r={error:.4f} m)')
#         ax.add_patch(circle)
        
#         # Add error annotation
#         ax.annotate(f'Error: {error:.4f} m', 
#                     xy=(self.x_positions[-1], self.y_positions[-1]),
#                     xytext=(10, 10), textcoords='offset points',
#                     bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7),
#                     arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
        
#         ax.set_xlabel('X Position (m)', fontsize=12)
#         ax.set_ylabel('Y Position (m)', fontsize=12)
#         ax.set_title('Odometry Trajectory with Reference Path and Error', fontsize=14, fontweight='bold')
#         ax.axis('equal')
#         ax.grid(True, alpha=0.3)
#         ax.legend(loc='best', fontsize=10)
        
#         # Add text box with statistics
#         textstr = f'Distance traveled: {np.sqrt((x_stop - x_start)**2 + (y_stop - y_start)**2):.4f} m\n'
#         textstr += f'Total error: {error:.4f} m\n'
#         textstr += f'Error percentage: {(error / np.sqrt((x_stop - x_start)**2 + (y_stop - y_start)**2) * 100):.2f}%'
#         props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
#         ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10,
#                 verticalalignment='top', bbox=props)
        
#         plt.tight_layout()
        
#         if save_path:
#             plt.savefig(save_path, dpi=300, bbox_inches='tight')
#             print(f"Plot saved to: {save_path}")
        
#         plt.show()

#         # print("last x odometry position:", self.x_positions[-1])
#         # print("last y odometry position:", self.y_positions[-1])


# def main(args=None):
#     rclpy.init(args=args)
#     analyzer = Node('node')

#     bag_path = BAG_PATH
    
#     odom_analyzer = Analyzer()

    
#     if odom_analyzer.read_bag(bag_path):
#         start_pixels, stop_pixels = odom_analyzer.extract_pixel_values()

#         print("\n=== Processing START image ===")
#         x_start, y_start = odom_analyzer.pixel_to_world(
#             start_pixels[0],    # origin pixel
#             start_pixels[1],    # x direction pixel
#             odom_analyzer.K,
#             odom_analyzer.H
#         )

#         print(f"Start world coordinates: x={x_start:.4f} m, y={y_start:.4f} m")
#         print("\n=== Processing STOP image ===")

        
#         x_stop, y_stop = odom_analyzer.pixel_to_world(
#             stop_pixels[0],   # origin pixel
#             stop_pixels[1],   # x direction pixel
#             odom_analyzer.K,
#             odom_analyzer.H
#         )
#         # ADD (X,Y) real position to the last position
#         x_stop = x_stop + DISTANCE
#         print(f"Stop world coordinates: x={x_stop:.4f} m, y={y_stop:.4f} m")

#         # error calculation
#         error = odom_analyzer.error_calculation(x_start, y_start, x_stop, y_stop)

#         print(f"odometry_x: {odom_analyzer.x_positions[0]}")
#         print(f"odometry_y: {odom_analyzer.y_positions[0]}")


#         # Plot results with all the information
#         save_path = os.path.join(os.path.dirname(bag_path), NAME + ".png")
#         odom_analyzer.plot_trajectory(x_start, y_start, x_stop, y_stop, error, save_path)
    
    
#     analyzer.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

