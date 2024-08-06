"""
Script Name: drone_search_and_center.py
Version: 1.0
Date: 2024-08-05
Description: This script uses MAVSDK for drone control and TensorFlow Lite for object detection to have a drone fly up 15 meters,
perform a 360-degree rotation to search for another drone, and zero out the horizontal and vertical theta values by adjusting its yaw and pitch.
"""

#!/usr/bin/env python3

import argparse
import asyncio
import sys
import time
import datetime
import csv
import os
import math
import numpy as np
import cv2
import mediapipe as mp
from sklearn.linear_model import LinearRegression
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from picamera2 import Picamera2
from mavsdk import System

# Global variables to calculate FPS
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
    csv_writer.writerow(['Date', 'Time', 'Width', 'Height', 'Confidence',
                        'Horizontal Distance', 'Vertical Distance', 'Distance'])

# Known distances for initial calibration
KNOWN_DISTANCES = [2.13, 1.83, 1.52, 1.22, 0.91]

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


def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
    global FPS, COUNTER, START_TIME, detection_result_list, horizontal_fov_degrees, vertical_fov_degrees, areas, distances

    # Calculate the FPS
    if COUNTER % fps_avg_frame_count == 0:
        FPS = fps_avg_frame_count / (time.time() - START_TIME)
        START_TIME = time.time()

    detection_result_list.append(result)
    COUNTER += 1

    # Center of the camera screen
    screen_center_x, screen_center_y = 960, 540

    for detection in result.detections:
        bbox = detection.bounding_box
        width = bbox.width
        height = bbox.height
        score = detection.categories[0].score

        bbox_center_x = bbox.origin_x + width / 2
        bbox_center_y = bbox.origin_y + height / 2

        horizontal_distance = bbox_center_x - screen_center_x
        vertical_distance = screen_center_y - bbox_center_y  # Invert the y-coordinate

        horizontal_theta = (horizontal_distance /
                            screen_center_x) * (horizontal_fov_degrees / 2)
        vertical_theta = (vertical_distance / screen_center_y) * \
            (vertical_fov_degrees / 2)

        area = width * height

        if len(distances) < len(KNOWN_DISTANCES):
            distances.append(KNOWN_DISTANCES[len(distances)])
            areas.append(area)

        slope, intercept = fit_trendline()
        distance_to_object = calculate_distance(
            area, slope, intercept) if slope is not None else "N/A"

        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        print(f"[{current_time}] Detected object bounding box - Width: {width} pixels, Height: {height} pixels, Confidence: {score:.2f}, Center: ({bbox_center_x}, {bbox_center_y}), Horizontal Distance: {horizontal_distance} pixels, Vertical Distance: {vertical_distance} pixels, Horizontal Theta: {horizontal_theta:.2f} degrees, Vertical Theta: {vertical_theta:.2f} degrees, Area: {area} pixels^2, Distance: {distance_to_object} meters")

        if log_data:
            date, time_with_ms = current_time.split(' ')
            csv_writer.writerow([date, time_with_ms, width, height, score,
                                horizontal_distance, vertical_distance, distance_to_object])

        return bbox, distance_to_object, horizontal_theta, vertical_theta


async def run_drone():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyAMA0:57600")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(1)
    flying_alt = absolute_altitude + 15.0
    await drone.action.goto_location(47.397606, 8.543060, flying_alt, 0)

    print("-- Starting 360-degree search")
    for i in range(36):
        await drone.action.set_yaw(10 * i)
        await asyncio.sleep(1)
        detection_result = await detect_drone()
        if detection_result:
            bbox, distance, horizontal_theta, vertical_theta = detection_result
            if horizontal_theta is not None and vertical_theta is not None:
                print(
                    f"Drone found. Horizontal Theta: {horizontal_theta:.2f}, Vertical Theta: {vertical_theta:.2f}")
                await zero_out_thetas(drone, horizontal_theta, vertical_theta)
                break

    print("Staying connected, press 'q' to return to launch")
    while True:
        if input() == 'q':
            await drone.action.return_to_launch()
            break
        await asyncio.sleep(1)


async def detect_drone():
    if detection_result_list:
        detection_result = detection_result_list[0]
        detection_result_list.clear()
        return save_result(detection_result, None, time.time_ns() // 1_000_000)
    return None


async def zero_out_thetas(drone, horizontal_theta, vertical_theta):
    print("Zeroing out thetas")
    print("Yawing to zero horizontal theta")
    await asyncio.sleep(1)
    for i in range(3, 0, -1):
        print(i)
        await asyncio.sleep(1)
    await drone.action.set_yaw(-horizontal_theta)
    print("Yaw zeroed")

    print("Adjusting pitch to zero vertical theta")
    await asyncio.sleep(1)
    for i in range(3, 0, -1):
        print(i)
        await asyncio.sleep(1)
    await drone.action.set_pitch(-vertical_theta)
    print("Pitch zeroed")
    print("Drone centered")


def run(model: str, max_results: int, score_threshold: float, camera_id: int, width: int, height: int) -> None:
    row_size = 50  # pixels
    left_margin = 24  # pixels
    text_color = (0, 0, 0)  # black
    font_size = 1
    font_thickness = 1

    detection_frame = None
    global horizontal_fov_degrees, vertical_fov_degrees
    horizontal_fov_radians = math.radians(horizontal_fov_degrees)
    vertical_fov_radians = math.radians(vertical_fov_degrees)

    base_options = python.BaseOptions(model_asset_path=model)
    options = vision.ObjectDetectorOptions(base_options=base_options,
                                           running_mode=vision.RunningMode.LIVE_STREAM,
                                           max_results=max_results, score_threshold=score_threshold,
                                           result_callback=save_result)
    detector = vision.ObjectDetector.create_from_options(options)

    try:
        while True:
            im = picam2.capture_array()
            image = cv2.resize(im, (1920, 1080))
            image = cv2.flip(image, -1)

            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB, data=rgb_image)

            detector.detect_async(mp_image, time.time_ns() // 1_000_000)

            fps_text = 'FPS = {:.1f}'.format(FPS)
            text_location = (left_margin, row_size)
            detection_frame = image.copy()

            cv2.putText(detection_frame, fps_text, text_location,
                        cv2.FONT_HERSHEY_PLAIN, font_size, text_color, font_thickness)

            cv2.imshow('Drone Detection', detection_frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break

    finally:
        cv2.destroyAllWindows()
        if log_data:
            csv_file.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--model', help='Path to the object detection model.', required=True)
    parser.add_argument('--maxResults', help='Max results of detection.',
                        required=False, default=3, type=int)
    parser.add_argument('--scoreThreshold', help='Score threshold for detection.',
                        required=False, default=0.5, type=float)
    parser.add_argument('--cameraId', help='Id of camera.',
                        required=False, default=0, type=int)
    parser.add_argument('--frameWidth', help='Width of frame to capture from camera.',
                        required=False, default=1920, type=int)
    parser.add_argument('--frameHeight', help='Height of frame to capture from camera.',
                        required=False, default=1080, type=int)
    args = parser.parse_args()

    # Running the object detection and drone control in separate asyncio tasks
    loop = asyncio.get_event_loop()
    loop.run_until_complete(asyncio.gather(
        loop.run_in_executor(None, run, args.model, args.maxResults,
                             args.scoreThreshold, args.cameraId, args.frameWidth, args.frameHeight),
        run_drone()
    ))
    loop.close()
