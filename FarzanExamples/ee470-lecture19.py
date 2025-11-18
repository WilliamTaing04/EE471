# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# Vision-Based Robotic Manipulation (EE 470), Lecture 19

import numpy as np
import cv2
import matplotlib.pyplot as plt
from pathlib import Path

def detect_red_blue(image_path):
    # Read image
    image_path = Path(image_path)
    image = cv2.imread(str(image_path))
    
    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Apply Gaussian Filter
    hsv = cv2.GaussianBlur(hsv, (5,5), 0)
    
    # Define HSV ranges for red and blue
    # OpenCV's hue range is from 1° to 180° instead of the common 1° to 360°
    # Red wraps around the hue spectrum, so we need two ranges
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # Blue range
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 255, 255])
    
    # Create masks
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Apply morphological operations to clean up the masks
    kernel = np.ones((5,5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    
    # Create output images with only red and blue objects
    red_result = cv2.bitwise_and(image, image, mask=red_mask)
    blue_result = cv2.bitwise_and(image, image, mask=blue_mask)
    
    # Combine red and blue detections into one image
    combined_mask = cv2.bitwise_or(red_mask, blue_mask)
    combined_result = cv2.bitwise_and(image, image, mask=combined_mask)
    
    # Display results
    plt.figure(figsize=(12, 8))
    
    plt.subplot(231)
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title('Original Image')
    plt.axis('off')
    
    plt.subplot(232)
    plt.imshow(red_mask, cmap='gray')
    plt.title('Red Mask')
    plt.axis('off')
    
    plt.subplot(233)
    plt.imshow(blue_mask, cmap='gray')
    plt.title('Blue Mask')
    plt.axis('off')
    
    plt.subplot(234)
    plt.imshow(cv2.cvtColor(red_result, cv2.COLOR_BGR2RGB))
    plt.title('Red Objects')
    plt.axis('off')
    
    plt.subplot(235)
    plt.imshow(cv2.cvtColor(blue_result, cv2.COLOR_BGR2RGB))
    plt.title('Blue Objects')
    plt.axis('off')
    
    plt.subplot(236)
    plt.imshow(cv2.cvtColor(combined_result, cv2.COLOR_BGR2RGB))
    plt.title('Combined Result')
    plt.axis('off')
    
    plt.tight_layout()
    plt.show()
    
    return red_mask, blue_mask, combined_result

def print_hsv_values(event, x, y, flags, param):
    """Mouse callback function to print HSV values"""
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_image = param
        hsv_value = hsv_image[y, x]
        print(f'HSV value at ({x},{y}): {hsv_value}')

def interactive_hsv_tuning(image_path):
    """Interactive window to help tune HSV values"""
    image_path = Path(image_path)
    image = cv2.imread(str(image_path))
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    cv2.namedWindow('HSV Tuning (ESC to EXIT)')
    cv2.setMouseCallback('HSV Tuning (ESC to EXIT)', print_hsv_values, hsv)
    
    while True:
        cv2.imshow('HSV Tuning (ESC to EXIT)', image)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key to exit
            break
    
    cv2.destroyAllWindows()

if __name__ == "__main__":

    script_dir = Path(__file__).parent
    image_path = script_dir / 'rubiks.jpg'
    
    # Use the following line to use interactive HSV value finder
    interactive_hsv_tuning(image_path)

    # Detect red and blue objects
    red_mask, blue_mask, result = detect_red_blue(image_path)
