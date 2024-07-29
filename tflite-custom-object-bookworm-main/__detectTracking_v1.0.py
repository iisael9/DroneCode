"""
Drone Control and Object Tracking System

Overview:
This script is designed to control a drone based on object detection using a camera and a TensorFlow Lite model. The drone will track objects, calculate their distance, and adjust its position based on detected angles. Additionally, it monitors the battery level and will return to launch if the battery drops below 50%.

Key Components:
1. **Camera Setup**:
   - Initialize the camera with a resolution of 1920x1080.
   - Capture images for object detection.

2. **Object Detection**:
   - Load a pre-trained TensorFlow Lite model for detecting objects.
   - Process each captured image to detect objects and obtain bounding box coordinates, confidence scores, and object sizes.

3. **Distance Calculation**:
   - Calculate the distance to detected objects based on bounding box area.
   - Use a linear regression model fitted with known distances to estimate object distance.

4. **Drone Control**:
   - Adjust the drone's yaw (horizontal angle) and pitch (vertical angle) based on the detected angles to keep the object centered.
   - Send commands to the drone using the PyMavlink library.

5. **Battery Monitoring**:
   - Periodically check the drone's battery level.
   - If the battery level falls below 50%, send a command to return the drone to its launch position.

6. **Logging**:
   - Optionally log detection data (e.g., object size, distance, confidence) to a CSV file.
   - User is prompted to enable or disable logging.

How it Works:
1. The script starts by initializing the camera and setting up the object detection model.
2. It enters a loop where it continuously captures frames from the camera and performs object detection.
3. For each detected object, it calculates the distance and adjusts the drone's position to keep the object centered.
4. It monitors the battery level and returns the drone to launch if necessary.
5. The detected data is optionally logged to a CSV file for further analysis.

Usage:
- Ensure the camera and drone are properly set up and connected.
- Adjust parameters such as model path, max results, score threshold, and camera resolution as needed.
- Run the script to start object tracking and drone control.
"""


from pymavlink import mavutil
import argparse
import time
import datetime
import csv
import os
import math
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from picamera2 import Picamera2
from sklearn.linear_model import LinearRegression

# Global variables
COUNTER, FPS = 0, 0
START_TIME = time.time()
fps_avg_frame_count = 10
detection_result_list = []
horizontal_fov_degrees = 66  # Field of view in degrees
vertical_fov_degrees = 49  # Assuming a vertical field of view

picam2 = Picamera2()
picam2.preview_configuration.main.size = (1920, 1080)  # Set to 1920x1080
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Define the path where you want to save the CSV file
csv_file_path = os.path.join(os.getcwd(), 'detection_data.csv')

# Function to ask the user if they want to log data
def ask_to_log():
    response = input("Do you want to log the data? (yes/no): ").strip().lower()
    return response == 'yes'

log_data = ask_to_log()

# Open the CSV file and write the header if logging is enabled
if log_data:
    csv_file = open(csv_file_path, mode='a', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Date', 'Time', 'Width', 'Height', 'Confidence', 'Horizontal Distance', 'Vertical Distance', 'Distance'])

# Known distances for initial calibration
KNOWN_DISTANCES = [2.13, 1.83, 1.52, 1.22, 0.91]  # Distances in meters (7ft, 6ft, 5ft, 4ft, 3ft)

# Initialize lists to store areas and corresponding distances
areas = []
distances = []

def fit_trendline():
    global areas, distances
    if len(areas) < 2:
        return None, None
    areas_np = np.array(areas).reshape(-1, 1)
    distances_np = np.array(distances)
    model = LinearRegression()
    model.fit(areas_np, distances_np)
    return model.coef_[0], model.intercept_

def calculate_distance(area, slope, intercept):
    return slope * area + intercept

def get_battery_level(master):
    battery_msg = master.recv_match(type='SYS_STATUS', blocking=True)
    if battery_msg:
        return battery_msg.battery_remaining
    return None

def return_to_launch(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0, 0, 0, 0)

def control_drone(horizontal_theta, vertical_theta, master, threshold=1.0):
    if abs(horizontal_theta) > threshold:
        if horizontal_theta > 0:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                abs(horizontal_theta), 1, 1, 0, 0, 0, 0)
        elif horizontal_theta < 0:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                abs(horizontal_theta), -1, 1, 0, 0, 0, 0)

    if abs(vertical_theta) > threshold:
        if vertical_theta > 0:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1, abs(vertical_theta), 0, 0, 0, 0, 0)
        elif vertical_theta < 0:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1, -abs(vertical_theta), 0, 0, 0, 0, 0)

def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
    global FPS, COUNTER, START_TIME, detection_result_list, horizontal_fov_degrees, vertical_fov_degrees, areas, distances

    # Calculate the FPS
    if COUNTER % fps_avg_frame_count == 0:
        FPS = fps_avg_frame_count / (time.time() - START_TIME)
        START_TIME = time.time()

    detection_result_list.append(result)
    COUNTER += 1

    # Center of the camera screen
    screen_center_x, screen_center_y = 960, 540  # Assuming the center of the resized frame (1920, 1080)

    # Print out the rectangle size and confidence score for each detected object with timestamp
    for detection in result.detections:
        bbox = detection.bounding_box
        width = bbox.width
        height = bbox.height
        score = detection.categories[0].score

        # Calculate the center of the bounding box
        bbox_center_x = bbox.origin_x + width / 2
        bbox_center_y = bbox.origin_y + height / 2

        # Calculate the distance from the center of the screen to the center of the bounding box
        horizontal_distance = bbox_center_x - screen_center_x 
        vertical_distance = screen_center_y - bbox_center_y  # Invert the y-coordinate

        # Calculate horizontal theta relative to the camera's FOV
        horizontal_theta = (horizontal_distance / screen_center_x) * (horizontal_fov_degrees / 2)

        # Calculate vertical theta relative to the camera's FOV
        vertical_theta = (vertical_distance / screen_center_y) * (vertical_fov_degrees / 2)

        # Calculate the area of the bounding box
        area = width * height

        # Append current area and distance to the lists for fitting
        if len(distances) < len(KNOWN_DISTANCES):
            distances.append(KNOWN_DISTANCES[len(distances)])
            areas.append(area)

        # Fit the trendline and calculate distance
        slope, intercept = fit_trendline()
        distance_to_object = calculate_distance(area, slope, intercept) if slope is not None else "N/A"

        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        print(f"[{current_time}] Detected object bounding box - Width: {width} pixels, Height: {height} pixels, Confidence: {score:.2f}, Center: ({bbox_center_x}, {bbox_center_y}), Horizontal Distance: {horizontal_distance} pixels, Vertical Distance: {vertical_distance} pixels, Horizontal Theta: {horizontal_theta:.2f} degrees, Vertical Theta: {vertical_theta:.2f} degrees, Area: {area} pixels^2, Distance: {distance_to_object} meters")

        # Write the data to the CSV file if logging is enabled
        if log_data:
            date, time_with_ms = current_time.split(' ')
            csv_writer.writerow([date, time_with_ms, width, height, score, horizontal_distance, vertical_distance, distance_to_object])

        # Return the bounding box, distance, and angles for displaying on the screen
        return bbox, distance_to_object, horizontal_theta, vertical_theta

def run(model: str, max_results: int, score_threshold: float, 
        camera_id: int, width: int, height: int) -> None:
    """Continuously run inference on images acquired from the camera.

    Args:
        model: Name of the TFLite object detection model.
        max_results: Max number of detection results.
        score_threshold: The score threshold of detection results.
        camera_id: The camera id to be passed to OpenCV.
        width: The width of the frame captured from the camera.
        height: The height of the frame captured from the camera.
    """

    # Visualization parameters
    row_size = 50  # pixels
    left_margin = 24  # pixels
    text_color = (0, 0, 0)  # black
    font_size = 1
    font_thickness = 1

    detection_frame = None

    # Camera FOV parameters
    global horizontal_fov_degrees, vertical_fov_degrees
    horizontal_fov_radians = math.radians(horizontal_fov_degrees)
    vertical_fov_radians = math.radians(vertical_fov_degrees)

    # Initialize the object detection model
    base_options = python.BaseOptions(model_asset_path=model)
    options = vision.ObjectDetectorOptions(base_options=base_options,
                                           running_mode=vision.RunningMode.LIVE_STREAM,
                                           max_results=max_results, score_threshold=score_threshold,
                                           result_callback=save_result)
    detector = vision.ObjectDetector.create_from_options(options)

    # Create the connection to the drone
    master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)
    master.wait_heartbeat()
    print("Heartbeat!")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 1, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    print('Armed!')

    while True:
        # Capture frame from the camera
        rgb_image = picam2.capture_array()
        if rgb_image is None:
            print("Error: No image captured.")
            continue

        # Perform object detection
        detection_result = detector.detect(rgb_image)
        detection_result_list.append(detection_result)

        # Check battery level
        battery_level = get_battery_level(master)
        if battery_level is not None:
            print(f"Battery level: {battery_level}%")
            if battery_level < 50:
                print("Battery below 50%, returning to launch")
                return_to_launch(master)
                break

        for detection in detection_result.detections:
            bbox, distance_to_object, horizontal_theta, vertical_theta = save_result(detection, rgb_image, time.time())
            if bbox:
                top_left = (int(bbox.origin_x), int(bbox.origin_y))
                bottom_right = (int(bbox.origin_x + bbox.width), int(bbox.origin_y + bbox.height))
                cv2.rectangle(rgb_image, top_left, bottom_right, (0, 255, 0), 2)
                label = f"{distance_to_object:.2f} m, θh: {horizontal_theta:.2f}, θv: {vertical_theta:.2f}"
                cv2.putText(rgb_image, label, (bbox.origin_x, bbox.origin_y - 10), cv2.FONT_HERSHEY_SIMPLEX, font_size, text_color, font_thickness)

                # Control the drone based on the detected angles
                control_drone(horizontal_theta, vertical_theta, master, threshold=1.0)

        # Display frame
        if detection_frame is None:
            detection_frame = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.imshow("Object Detection", rgb_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    if log_data:
        csv_file.close()

if __name__ == "__main__":
    # Set parameters
    model_path = "/home/rpi5/Github/DroneCode/tflite-custom-object-bookworm-main/notBest.tflite"
    max_results = 5
    score_threshold = 0.5
    camera_id = 0
    width, height = 1920, 1080

    run(model_path, max_results, score_threshold, camera_id, width, height)
