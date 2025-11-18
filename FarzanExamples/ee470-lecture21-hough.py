# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# Vision-Based Robotic Manipulation (EE 470), Lecture 21

import cv2
import numpy as np
from pathlib import Path


def detect_circles_hough(image_path):
    # Read image
    image_path = Path(image_path)
    image = cv2.imread(str(image_path))
    original = image.copy()
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 2)
    
    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,              # Inverse ratio of accumulator resolution
        minDist=200,       # Minimum distance between centers
        param1=50,         # Upper threshold for edge detector
        param2=30,         # Threshold for center detection
        minRadius=100,     # Minimum radius
        maxRadius=150      # Maximum radius
    )
    
    # Draw detected circles
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            radius = i[2]
            
            # Draw the outer circle
            cv2.circle(image, center, radius, (0, 255, 0), 2)
            # Draw the center point
            cv2.circle(image, center, 3, (0, 0, 255), -1)
            # Add radius information
            cv2.putText(image, f'R: {radius}', 
                       (center[0]-20, center[1]-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # Display results
    cv2.imshow('Original', original)
    cv2.imshow('Grayscale', gray)
    cv2.imshow('Blurred', blurred)
    cv2.imshow('Detected Circles (Hough)', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Run the detection
script_dir = Path(__file__).parent
image_path = script_dir / 'coins.jpg'
detect_circles_hough(image_path)
