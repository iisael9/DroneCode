import argparse
import sys
import time
import datetime
import csv
import os

import cv2
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from utils import visualize  # Assuming this function draws bounding boxes and labels
from picamera2 import Picamera2

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Define the path where you want to save the CSV file
csv_file_path = os.path.join(os.getcwd(), 'detection_data.csv')

# Open the CSV file and write the header
csv_file = open(csv_file_path, mode='a', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Date', 'Time', 'Width', 'Height', 'Confidence', 'Center X', 'Center Y', 'Grid'])

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
    fps_avg_frame_count = 10

    detection_frame = None
    detection_result_list = []

    def get_grid_section(center_x, center_y, width, height):
        # Define the grid boundaries
        grid_width = width / 3
        grid_height = height / 3

        if center_x < grid_width:
            col = 1
        elif center_x < 2 * grid_width:
            col = 2
        else:
            col = 3

        if center_y < grid_height:
            row = 1
        elif center_y < 2 * grid_height:
            row = 2
        else:
            row = 3

        return f"Row {row}, Column {col}"

    def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
        global FPS, COUNTER, START_TIME

        # Calculate the FPS
        if COUNTER % fps_avg_frame_count == 0:
            FPS = fps_avg_frame_count / (time.time() - START_TIME)
            START_TIME = time.time()

        detection_result_list.append(result)
        COUNTER += 1

        # Print out the rectangle size, confidence score, and center coordinates for each detected object with timestamp
        for detection in result.detections:
            bbox = detection.bounding_box
            width = bbox.width
            height = bbox.height
            score = detection.categories[0].score  
            center_x = bbox.origin_x + width / 2
            center_y = bbox.origin_y + height / 2
            current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            grid_section = get_grid_section(center_x, center_y, 640, 480)
            print(f"[{current_time}] Detected object bounding box - Width: {width} pixels, Height: {height} pixels, Confidence: {score:.2f}, Center X: {center_x}, Center Y: {center_y}, Grid: {grid_section}")
            
            # Write the data to the CSV file
            date, time_with_ms = current_time.split(' ')
            csv_writer.writerow([date, time_with_ms, width, height, score, center_x, center_y, grid_section])

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
            image = cv2.resize(im, (640, 480))
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

            # Draw the grid
            grid_color = (0, 255, 0)  # Green
            grid_thickness = 1
            for i in range(1, 3):
                cv2.line(current_frame, (int(width * i / 3), 0), (int(width * i / 3), height), grid_color, grid_thickness)
                cv2.line(current_frame, (0, int(height * i / 3)), (width, int(height * i / 3)), grid_color, grid_thickness)

            if detection_result_list:
                current_frame = visualize(current_frame, detection_result_list[0])
                detection_frame = current_frame
                detection_result_list.clear()

            if detection_frame is not None:
                cv2.imshow('object_detection', detection_frame)

            # Stop the program if the ESC key is pressed.
            if cv2.waitKey(1) == 27:
                break
    finally:
        # Ensure the CSV file is closed properly
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
        required=False,
        type=float,
        default=0.25)
    parser.add_argument(
        '--cameraId', help='Id of camera.', required=False, type=int, default=0)
    parser.add_argument(
        '--frameWidth',
        help='Width of frame to capture from camera.',
        required=False, type=int, default=640)
    parser.add_argument(
        '--frameHeight',
        help='Height of frame to capture from camera.',
        required=False, type=int, default=480)
    args = parser.parse_args()

    run(args.model, int(args.maxResults),
        args.scoreThreshold, int(args.cameraId), args.frameWidth, args.frameHeight)

if __name__ == '__main__':
    main()
