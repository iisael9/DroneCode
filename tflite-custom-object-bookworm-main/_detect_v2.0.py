import argparse
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

# from utils import visualize  # Assuming this function draws bounding boxes and labels
from picamera2 import Picamera2

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

    try:
        # Continuously capture images from the camera and run inference
        while True:
            im = picam2.capture_array()
            image = cv2.resize(im, (1920, 1080))
            image = cv2.flip(image, -1)

            # Convert the image from BGR to RGB as required by the TFLite model.
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

            # Run object detection using the model.
            detector.detect_async(mp_image, time.time_ns() // 1_000_000)

            # Show the FPS
            fps_text = 'FPS = {:.1f}'.format(FPS)
            text_location = (left_margin, row_size)
            current_frame = image
            cv2.putText(current_frame, fps_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                        font_size, text_color, font_thickness, cv2.LINE_AA)

            # Draw a 4-quadrant grid with the center being zero
            center_x, center_y = 960, 540  # Assuming the center of the resized frame (1920, 1080)
            cv2.line(current_frame, (center_x, 0), (center_x, 1080), (255, 255, 255), 1)
            cv2.line(current_frame, (0, center_y), (1920, center_y), (255, 255, 255), 1)
            cv2.putText(current_frame, '(0, 0)', (center_x + 5, center_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)

            bbox, distance, horizontal_theta, vertical_theta = None, None, None, None
            if detection_result_list:
                detection_result = detection_result_list[0]
                for detection in detection_result.detections:
                    bbox = detection.bounding_box
                    width = bbox.width
                    height = bbox.height
                    bbox_center_x = bbox.origin_x + width / 2
                    bbox_center_y = bbox.origin_y + height / 2

                    # Adjust coordinates to make the center of the screen (0, 0)
                    adjusted_bbox_center_x = bbox_center_x - center_x
                    adjusted_bbox_center_y = center_y - bbox_center_y  # Invert the y-coordinate

                    # Calculate horizontal theta relative to the camera's FOV
                    horizontal_theta = (adjusted_bbox_center_x / center_x) * (horizontal_fov_degrees / 2)

                    # Calculate vertical theta relative to the camera's FOV
                    vertical_theta = (adjusted_bbox_center_y / center_y) * (vertical_fov_degrees / 2)

                    # Show the coordinates of the center of the bounding box and angles
                    cv2.putText(current_frame, f'({adjusted_bbox_center_x}, {adjusted_bbox_center_y})', 
                                (int(bbox_center_x), int(bbox_center_y)), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5, (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.putText(current_frame, f'Horiz. Theta: {horizontal_theta:.2f}', (center_x + 5, center_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(current_frame, f'Vert. Theta: {vertical_theta:.2f}', (center_x + 5, center_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5, (255, 255, 255), 1, cv2.LINE_AA)

                    # Get the distance to the object
                    area = width * height
                    slope, intercept = fit_trendline()
                    distance = calculate_distance(area, slope, intercept) if slope is not None else "N/A"

                detection_frame = current_frame
                detection_result_list.clear()

            if detection_frame is not None:
                # Create a sidebar to display distance, horizontal theta, and vertical theta
                sidebar_width = 200
                sidebar = np.zeros((1080, sidebar_width, 3), dtype=np.uint8)
                cv2.rectangle(sidebar, (0, 0), (sidebar_width, 1080), (0, 0, 0), -1)

                # Display the distance, horizontal theta, and vertical theta in the sidebar
                if bbox is not None and distance is not None and horizontal_theta is not None and vertical_theta is not None:
                    info_text = [
                        f'Distance: {distance:.2f}m' if isinstance(distance, float) else f'Distance: {distance}',
                        f'Horiz. Theta: {horizontal_theta:.2f}°',
                        f'Vert. Theta: {vertical_theta:.2f}°'
                    ]
                    for i, text in enumerate(info_text):
                        cv2.putText(sidebar, text, (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.7, (255, 255, 255), 2, cv2.LINE_AA)  # White text

                    # Draw bounding box in magenta purple
                    cv2.rectangle(detection_frame, (int(bbox.origin_x), int(bbox.origin_y)), 
                                  (int(bbox.origin_x + bbox.width), int(bbox.origin_y + bbox.height)), 
                                  (255, 0, 255), 2)

                    # Draw line from the camera (center of the screen) to the center of the bounding box in orange
                    cv2.line(detection_frame, (center_x, center_y), (int(bbox_center_x), int(bbox_center_y)), 
                             (0, 165, 255), 2)

                # Combine the sidebar with the detection frame
                combined_frame = np.hstack((detection_frame, sidebar))

                cv2.imshow('object_detection', combined_frame)

            # Stop the program if the ESC key is pressed.
            if cv2.waitKey(1) == 27:
                break
    finally:
        # Ensure the CSV file is closed properly if logging is enabled
        if log_data:
            csv_file.close()
        detector.close()
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--model',
        help='Path of the object detection model.',
        required=False,
        default='notBest.tflite')
    parser.add_argument(
        '--maxResults',
        help='Max number of detection results.',
        required=False,
        default=5)
    parser.add_argument(
        '--scoreThreshold',
        help='The score threshold of detection results.',
        required=False, type=float, default=0.25)
    parser.add_argument(
        '--cameraId', help='Id of camera.', required=False, type=int, default=0)
    parser.add_argument(
        '--frameWidth',
        help='Width of frame to capture from camera.',
        required=False, type=int, default=1920)  # Set to 1920
    parser.add_argument(
        '--frameHeight',
        help='Height of frame to capture from camera.',
        required=False, type=int, default=1080)  # Set to 1080
    args = parser.parse_args()

    run(args.model, int(args.maxResults),
        args.scoreThreshold, int(args.cameraId), args.frameWidth, args.frameHeight)

if __name__ == '__main__':
    main()