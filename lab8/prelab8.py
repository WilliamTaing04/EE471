# EE471 Prelab 8 11/13/2025
# William Taing wtaing@calpoly.edu
# Description: for Prelab8

import numpy as np
import cv2
import matplotlib.pyplot as plt
from pathlib import Path


def detect_balls(image_path):
    # Read Image
    image_path = Path(image_path)
    image = cv2.imread(str(image_path))
    original = image.copy()

    # Brightness/Contrast
    image = cv2.convertScaleAbs(image, alpha=1.2, beta=30)

    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define HSV ranges for red and blue (1°-180°)    
    # Red
    lower_red1 = np.array([0, 75, 75])
    upper_red1 = np.array([11, 255, 255])
    lower_red2 = np.array([160, 75, 75])
    upper_red2 = np.array([180, 255, 255])  

    # Blue
    lower_blue = np.array([95, 75, 75])
    upper_blue = np.array([135, 255, 255])

    # Yellow
    lower_yellow = np.array([27, 100, 100])
    upper_yellow = np.array([40, 255, 255])

    # Orange
    lower_orange = np.array([10, 50, 50])
    upper_orange = np.array([20, 255, 255])

    # Create masks
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Apply morphological operations to clean up the masks
    kernel = np.ones((5,5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
    
    # Create output images with only red and blue objects
    red_result = cv2.bitwise_and(image, image, mask=red_mask)
    blue_result = cv2.bitwise_and(image, image, mask=blue_mask)
    yellow_result = cv2.bitwise_and(image, image, mask=yellow_mask)
    orange_result = cv2.bitwise_and(image, image, mask=orange_mask)

    # Convert to grayscale
    red_gray = cv2.cvtColor(red_result, cv2.COLOR_BGR2GRAY)
    blue_gray = cv2.cvtColor(blue_result, cv2.COLOR_BGR2GRAY)
    yellow_gray = cv2.cvtColor(yellow_result, cv2.COLOR_BGR2GRAY)
    orange_gray = cv2.cvtColor(orange_result, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    red_gray_blurred = cv2.GaussianBlur(red_gray, (5, 5), 0)
    blue_gray_blurred = cv2.GaussianBlur(blue_gray, (5, 5), 0)
    yellow_gray_blurred = cv2.GaussianBlur(yellow_gray, (5, 5), 0)
    orange_gray_blurred = cv2.GaussianBlur(orange_gray, (5, 5), 0)

    # Otsu's thresholding
    _, red_thresh_otsu = cv2.threshold(red_gray_blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, blue_thresh_otsu = cv2.threshold(blue_gray_blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, yellow_thresh_otsu = cv2.threshold(yellow_gray_blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, orange_thresh_otsu = cv2.threshold(orange_gray_blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Find contours on the edge image
    red_contours, _ = cv2.findContours(red_thresh_otsu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv2.findContours(blue_thresh_otsu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    yellow_contours, _ = cv2.findContours(yellow_thresh_otsu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    orange_contours, _ = cv2.findContours(orange_thresh_otsu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create debug image for contours
    red_contour_debug = original.copy()
    blue_contour_debug = original.copy()
    yellow_contour_debug = original.copy()
    orange_contour_debug = original.copy()
    cv2.drawContours(red_contour_debug, red_contours, -1, (0,255,0), 2)
    cv2.drawContours(blue_contour_debug, blue_contours, -1, (0,255,0), 2)
    cv2.drawContours(yellow_contour_debug, yellow_contours, -1, (0,255,0), 2)
    cv2.drawContours(orange_contour_debug, orange_contours, -1, (0,255,0), 2)


    # Process each contour
    min_area = 1000  # Adjust based on your image
    min_circularity = 0.5  # Lower threshold for circularity

    # Red contours
    for contour in red_contours:
        # Calculate area and perimeter
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        # Filter small contours and print area for debugging
        # print(f"Contour area: {area}")
        if area < min_area:
            continue
        
        # Calculate circularity
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        # print(f"Circularity: {circularity}")
        
        # If shape is roughly circular
        if circularity > min_circularity:
            # Calculate moments
            M = cv2.moments(contour)
            if M["m00"] != 0:
                # Calculate centroid
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate radius from area
                radius = int(np.sqrt(area / np.pi))
                
                # Draw the outer circle
                cv2.circle(original, (cx, cy), radius, (0, 255, 0), 2)
                # Draw the center point
                cv2.circle(original, (cx, cy), 3, (0, 0, 255), -1)
                
                # Add measurements
                cv2.putText(original, f'Area: {area:.0f}', 
                          (cx-20, cy-40),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Circ: {circularity:.2f}', 
                          (cx-20, cy-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Color: Red', 
                          (cx-20, cy-60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # Terminal Print
                print(f"Red: {cx}, {cy}")
    
    # Blue contours
    for contour in blue_contours:
        # Calculate area and perimeter
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        # Filter small contours and print area for debugging
        # print(f"Contour area: {area}")
        if area < min_area:
            continue
        
        # Calculate circularity
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        # print(f"Circularity: {circularity}")
        
        # If shape is roughly circular
        if circularity > min_circularity:
            # Calculate moments
            M = cv2.moments(contour)
            if M["m00"] != 0:
                # Calculate centroid
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate radius from area
                radius = int(np.sqrt(area / np.pi))
                
                # Draw the outer circle
                cv2.circle(original, (cx, cy), radius, (0, 255, 0), 2)
                # Draw the center point
                cv2.circle(original, (cx, cy), 3, (0, 0, 255), -1)
                
                # Add measurements
                cv2.putText(original, f'Area: {area:.0f}', 
                          (cx-20, cy-40),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Circ: {circularity:.2f}', 
                          (cx-20, cy-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Color: Blue', 
                          (cx-20, cy-60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # Terminal Print
                print(f"Blue: {cx}, {cy}")
                
    # Yellow contours
    for contour in yellow_contours:
        # Calculate area and perimeter
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        # Filter small contours and print area for debugging
        # print(f"Contour area: {area}")
        if area < min_area:
            continue
        
        # Calculate circularity
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        # print(f"Circularity: {circularity}")
        
        # If shape is roughly circular
        if circularity > min_circularity:
            # Calculate moments
            M = cv2.moments(contour)
            if M["m00"] != 0:
                # Calculate centroid
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate radius from area
                radius = int(np.sqrt(area / np.pi))
                
                # Draw the outer circle
                cv2.circle(original, (cx, cy), radius, (0, 255, 0), 2)
                # Draw the center point
                cv2.circle(original, (cx, cy), 3, (0, 0, 255), -1)
                
                # Add measurements
                cv2.putText(original, f'Area: {area:.0f}', 
                          (cx-20, cy-40),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Circ: {circularity:.2f}', 
                          (cx-20, cy-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Color: Yellow', 
                          (cx-20, cy-60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # Terminal Print
                print(f"Yellow: {cx}, {cy}")
                
    # Orange contours
    for contour in orange_contours:
        # Calculate area and perimeter
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        # Filter small contours and print area for debugging
        # print(f"Contour area: {area}")
        if area < min_area:
            continue
        
        # Calculate circularity
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        # print(f"Circularity: {circularity}")
        
        # If shape is roughly circular
        if circularity > min_circularity:
            # Calculate moments
            M = cv2.moments(contour)
            if M["m00"] != 0:
                # Calculate centroid
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate radius from area
                radius = int(np.sqrt(area / np.pi))
                
                # Draw the outer circle
                cv2.circle(original, (cx, cy), radius, (0, 255, 0), 2)
                # Draw the center point
                cv2.circle(original, (cx, cy), 3, (0, 0, 255), -1)
                
                # Add measurements
                cv2.putText(original, f'Area: {area:.0f}', 
                          (cx-20, cy-40),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Circ: {circularity:.2f}', 
                          (cx-20, cy-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Color: Orange', 
                          (cx-20, cy-60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # Terminal Print
                print(f"Orange: {cx}, {cy}")




    # Display Images
    # cv2.imshow('original', original)
    # cv2.imshow('image', image)
    # cv2.imshow('red', red_result)
    # cv2.imshow('blue', blue_result)
    # cv2.imshow('yellow', yellow_result)
    # cv2.imshow('orange', orange_result)
    # cv2.imshow('red Otsu Threshold', red_thresh_otsu)
    # cv2.imshow('blue Otsu Threshold', blue_thresh_otsu)
    # cv2.imshow('yellow Otsu Threshold', yellow_thresh_otsu)
    # cv2.imshow('orange Otsu Threshold', orange_thresh_otsu)
    # cv2.imshow('red contour', red_contour_debug)
    # cv2.imshow('blue contour', blue_contour_debug)
    # cv2.imshow('yellow contour', yellow_contour_debug)
    # cv2.imshow('orange contour', orange_contour_debug)
    cv2.imshow('Detected Circles (Contours)', original)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    script_dir = Path(__file__).parent
    image_path = script_dir / "image_prelab8.jpg"   # Get file path
    detect_balls(image_path)

    
    
  

    