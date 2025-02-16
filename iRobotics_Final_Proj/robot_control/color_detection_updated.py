#!/usr/bin/env python

#
import cv2
import numpy as np
import openni  # Import the OpenNI2 wrapper

# Define fixed HSV color ranges for Neon Green, Cyan, and Magenta
HSV_RANGES = {
    "Neon Green": ((50, 100, 100), (70, 255, 255)),
    "Cyan": ((80, 100, 100), (100, 255, 255)),
    "Magenta": ((140, 100, 100), (160, 255, 255))
}

def preprocess_image_for_stars(hsv_image):
    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(hsv_image, (5, 5), 0)
    # Define a range for bright colors to include all possible star colors
    bright_lower_bound = np.array([0, 50, 50])
    bright_upper_bound = np.array([180, 255, 255])
    # Create a mask for bright colors
    mask = cv2.inRange(blurred, bright_lower_bound, bright_upper_bound)
    # Use morphological operations to clean up the mask
    kernel = np.ones((3, 3), np.uint8)
    cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    return cleaned_mask

def find_star_contours_with_preprocessing(hsv_image):
    preprocessed_mask = preprocess_image_for_stars(hsv_image)
    # Find contours on the preprocessed mask
    contours, _ = cv2.findContours(preprocessed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Filter for star-shaped contours
    star_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]
    return star_contours

def detect_color(hsv_image, contour):
    # Create a mask for the contour
    mask = np.zeros(hsv_image.shape[:2], dtype="uint8")
    cv2.drawContours(mask, [contour], -1, 255, -1)
    # Calculate the mean color of the contour area
    mean_val = cv2.mean(hsv_image, mask=mask)
    mean_color = mean_val[:3]
    for color_name, (lower, upper) in HSV_RANGES.items():
        # Check if the mean color is within the specific range
        if all(lower[i] <= mean_color[i] <= upper[i] for i in range(3)):
            return color_name
    return "Unknown"

def sort_contours(contours):
    bounding_boxes = [cv2.boundingRect(c) for c in contours]
    (contours, bounding_boxes) = zip(*sorted(zip(contours, bounding_boxes),
                                             key=lambda b: b[1][0], reverse=False))
    return contours

def process_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    stars = find_star_contours_with_preprocessing(hsv)
    sorted_stars = sort_contours(stars)

    for star in sorted_stars:
        color = detect_color(hsv, star)
        cv2.drawContours(frame, [star], -1, (0, 255, 0), 3)
        x, y, w, h = cv2.boundingRect(star)
        cv2.putText(frame, color, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame

# Initialize OpenNI2 with error handling
try:
    openni.initialize()
except openni.OpenNIError as e:
    print(f"Failed to initialize OpenNI2: {e}")
    exit(1)

# Attempt to open the device
try:
    dev = openni.Device.open_any()
except openni.OpenNIError as e:
    print(f"Failed to open the device: {e}")
    openni.unload()
    exit(1)

# Attempt to create a VideoStream for RGB images
try:
    rgb_stream = dev.create_color_stream()
    rgb_stream.start()
except openni.OpenNIError as e:
    print(f"Failed to start color stream: {e}")
    dev.close()
    openni.unload()
    exit(1)

while True:
    try:
        frame = rgb_stream.read_frame()
        frame_data = frame.get_buffer_as_uint8()
    except openni.OpenNIError as e:
        print(f"Failed to read frame: {e}")
        break

    img = np.frombuffer(frame_data, dtype=np.uint8)
    img.shape = (1, frame.height, frame.width, 3)
    img = img[0]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    processed_frame = process_frame(img)

    cv2.imshow('Processed Frame', processed_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

rgb_stream.stop()
dev.close()
openni.unload()
cv2.destroyAllWindows()
