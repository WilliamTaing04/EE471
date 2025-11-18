# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# Vision-Based Robotic Manipulation (EE 470), Lecture 21

import cv2
import numpy as np
from pathlib import Path


def detect_circles_contours(image_path):
    # Read image
    image_path = Path(image_path)
    image = cv2.imread(str(image_path))
    original = image.copy()
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Try different thresholding methods
    # 1. Simple binary threshold
    _, thresh_binary = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)
    
    # 2. Otsu's thresholding
    _, thresh_otsu = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # Choose the best threshold (try each one)
    thresh = thresh_binary # thresh_otsu  # Change this to try different thresholds
    
    # Clean up mask with morphological operations
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours on the edge image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Create debug image for contours
    contour_debug = image.copy()
    cv2.drawContours(contour_debug, contours, -1, (0,255,0), 2)
    
    # Process each contour
    min_area = 50000  # Adjust based on your image
    min_circularity = 0.5  # Lower threshold for circularity
    
    for contour in contours:
        # Calculate area and perimeter
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        # Filter small contours and print area for debugging
        print(f"Contour area: {area}")
        if area < min_area:
            continue
        
        # Calculate circularity
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        print(f"Circularity: {circularity}")
        
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
                cv2.circle(image, (cx, cy), radius, (0, 255, 0), 2)
                # Draw the center point
                cv2.circle(image, (cx, cy), 3, (0, 0, 255), -1)
                
                # Add measurements
                cv2.putText(image, f'Area: {area:.0f}', 
                          (cx-20, cy-40),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(image, f'Circ: {circularity:.2f}', 
                          (cx-20, cy-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # Display all intermediate steps for debugging
    cv2.imshow('Original', original)
    cv2.imshow('Grayscale', gray)
    cv2.imshow('Blurred', blurred)
    cv2.imshow('Binary Threshold', thresh_binary)
    cv2.imshow('Otsu Threshold', thresh_otsu)
    cv2.imshow('Mask after Morphology', mask)
    cv2.imshow('Contours Debug', contour_debug)
    cv2.imshow('Detected Circles (Contours)', image)
    
    key = cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return key

# Run the detection with ability to try different thresholds
script_dir = Path(__file__).parent
image_path = script_dir / 'coins.jpg'
detect_circles_contours(image_path)