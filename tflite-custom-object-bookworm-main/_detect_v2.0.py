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
from picamera2 import Picamera2

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()
fps_avg_frame_count = 10
detection_result_list = []
horizontal_fov_degrees = 66  # Field of view in degrees
vertical_fov_degrees = 49  # Assuming a vertical field of view

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1920, 1080)  # Set to 1920x1080
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# CSV file path
csv_file_path = os.path.join(os.getcwd(), 'detection_data.csv')

# Ask the user if they want to log data


def ask_to_log():
    response = input("Do you want to log the data? (yes/no): ").strip().lower()
    return response == 'yes'


log_data = ask_to_log()

# Open the CSV file and write the header if logging is enabled
csv_writer = None
if log_data:
    csv_file = open(csv_file_path, mode='a', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Date', 'Time', 'Width', 'Height', 'Confidence',
                        'Horizontal Distance', 'Vertical Distance', 'Distance'])

# Known distances for initial calibration
# Distances in meters (7ft, 6ft, 5ft, 4ft, 3ft)
KNOWN_DISTANCES = [2.13, 1.83, 1.52, 1.22, 0.91]

# Initialize lists to store areas and corresponding distances
areas = []
distances = []


def fit_trendline():
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
    # Assuming the center of the resized frame (1920, 1080)
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


def draw_fps(image):
    fps_text = 'FPS = {:.1f}'.format(FPS)
    text_location = (24, 50)
    cv2.putText(image, fps_text, text_location,
                cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)


def draw_grid(image):
    center_x, center_y = 960, 540
    cv2.line(image, (center_x, 0), (center_x, 1080), (255, 255, 255), 1)
    cv2.line(image, (0, center_y), (1920, center_y), (255, 255, 255), 1)
    cv2.putText(image, '(0, 0)', (center_x + 5, center_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def display_info_sidebar(image, bbox, distance, horizontal_theta, vertical_theta):
    sidebar_width = 200
    sidebar = np.zeros((1080, sidebar_width, 3), dtype=np.uint8)
    cv2.rectangle(sidebar, (0, 0), (sidebar_width, 1080), (0, 0, 0), -1)

    if bbox is not None and distance is not None and horizontal_theta is not None and vertical_theta is not None:
        info_text = [
            f'Distance: {distance:.2f}m' if isinstance(
                distance, float) else f'Distance: {distance}',
            f'Horiz. Theta: {horizontal_theta:.2f}°',
            f'Vert. Theta: {vertical_theta:.2f}°'
        ]
        for i, text in enumerate(info_text):
            cv2.putText(sidebar, text, (10, 30 + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.rectangle(image, (int(bbox.origin_x), int(bbox.origin_y)), (int(
            bbox.origin_x + bbox.width), int(bbox.origin_y + bbox.height)), (255, 0, 255), 2)
        cv2.line(image, (960, 540), (int(bbox.origin_x + bbox.width / 2),
                 int(bbox.origin_y + bbox.height / 2)), (0, 165, 255), 2)

    combined_frame = np.hstack((image, sidebar))
    cv2.imshow('object_detection', combined_frame)


def run(model: str, max_results: int, score_threshold: float, camera_id: int, width: int, height: int) -> None:
    detection_frame = None
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

            draw_fps(image)
            draw_grid(image)

            bbox, distance, horizontal_theta, vertical_theta = None, None, None, None
            if detection_result_list:
                detection_result = detection_result_list[0]
                for detection in detection_result.detections:
                    bbox = detection.bounding_box
                    width = bbox.width
                    height = bbox.height
                    bbox_center_x = bbox.origin_x + width / 2
                    bbox_center_y = bbox.origin_y + height / 2

                    adjusted_bbox_center_x = bbox_center_x - 960
                    adjusted_bbox_center_y = 540 - bbox_center_y

                    horizontal_theta = (
                        adjusted_bbox_center_x / 960) * (horizontal_fov_degrees / 2)
                    vertical_theta = (adjusted_bbox_center_y /
                                      540) * (vertical_fov_degrees / 2)

                    area = width * height

                    slope, intercept = fit_trendline()
                    distance = calculate_distance(
                        area, slope, intercept) if slope is not None else "N/A"

            display_info_sidebar(image, bbox, distance,
                                 horizontal_theta, vertical_theta)

            if cv2.waitKey(1) & 0xFF == 27:
                break

    except Exception as e:
        print(f'Error occurred: {e}')
        if log_data:
            csv_file.close()

    if log_data:
        csv_file.close()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Run live object detection with distance calculation")
    parser.add_argument('--model', help='Path to the object detection model.', required=True)
    parser.add_argument('--maxResults', help='Maximum number of detection results', default=3, type=int)
    parser.add_argument('--scoreThreshold', help='Detection score threshold', default=0.5, type=float)
    parser.add_argument('--cameraId', help='ID of the camera to use', default=0, type=int)
    parser.add_argument('--width', help='Width of the frames to process', default=1920, type=int)
    parser.add_argument('--height', help='Height of the frames to process', default=1080, type=int)
    args = parser.parse_args()

    run(args.model, args.maxResults, args.scoreThreshold, args.cameraId, args.width, args.height)

if __name__ == '__main__':
    main()

