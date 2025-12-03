"""
Lab 8 (Final Project): Vision-Guided Robotic Pick-and-Place Sorting System
Team: Group 3
Members: Agustine Hernandez, William Taing

This script implements a complete robotic sorting system that:
1. Detects colored balls using computer vision
2. Localizes them in 3D space using camera-robot calibration
3. Plans smooth trajectories to pick up each ball
4. Sorts them into color-coded bins

System Architecture:
    Detection → Localization → Motion Planning → Execution → Repeat
"""

import numpy as np
import cv2
import time
import pickle
from pathlib import Path

from classes.Robot import Robot
from classes.Realsense import Realsense
from classes.TrajPlanner import TrajPlanner

# ============================================================================
# CONFIGURATION PARAMETERS
# ============================================================================

# Physical parameters
BALL_RADIUS = 15  # Physical radius of balls in millimeters

# Motion control parameters
TRAJECTORY_TIME = 2.5  # Time for each trajectory segment in seconds
NUM_POINTS = 100       # Number of waypoints in each trajectory

# Workspace safety bounds (millimeters, in robot frame)
# TODO: Adjust these based on your setup to prevent collisions
X_MIN, X_MAX = 50, 230   # Forward/backward limits
Y_MIN, Y_MAX = -150, 150 # Left/right limits

# Home position: [x, y, z, pitch] in mm and degrees
# This position should give the camera a clear view of the workspace
HOME_POSITION = [100, 0, 220, -15]

# Sorting bin locations: [x, y, z, pitch] in mm and degrees
# TODO: Adjust these positions based on your physical bin locations
BINS = {
    'red': [0, -220, 150, -40],
    'orange': [120, -220, 150, -40],
    'blue': [0, 220, 150, -45],
    'yellow': [120, 220, 150, -45]
}

# ============================================================================
# COMPUTER VISION: BALL DETECTION AND POSE ESTIMATION
# ============================================================================

def get_ball_pose(corners: np.ndarray, intrinsics: any, radius: float) -> tuple:
    """
    Estimate the 3D pose of a detected sphere using the Perspective-n-Point (PnP) algorithm.
    
    The PnP algorithm finds the position and orientation of an object by matching:
    - Known 3D points on the object (object_points)
    - Corresponding 2D points in the image (image_points)
    
    For a sphere, we create artificial "corner" points on the sphere's visible boundary
    to establish these correspondences.
    
    Args:
        corners: A 4x2 array of circle boundary points [top, bottom, left, right]
        intrinsics: Camera intrinsic parameters from RealSense
        radius: The sphere's physical radius in millimeters
    
    Returns:
        tuple: (rotation_matrix, translation_vector)
            - rotation_matrix: 3x3 matrix (not used for spheres, but returned by PnP)
            - translation_vector: 3x1 vector giving sphere center in camera frame (mm)
    
    Raises:
        RuntimeError: If PnP algorithm fails to find a solution
    """
    # ==========================================================================
    # TODO: Define 3D object points on the sphere
    # ==========================================================================
    # Hint: Create 4 points on the sphere's "equator" as viewed from camera
    # These should correspond to the top, bottom, left, and right boundary points
    # Example structure:
    #   Point 1: [-radius, 0, 0]  (left edge of sphere)
    #   Point 2: [+radius, 0, 0]  (right edge of sphere)
    #   Point 3: [0, +radius, 0]  (top edge of sphere)
    #   Point 4: [0, -radius, 0]  (bottom edge of sphere)
    # YOUR CODE HERE
    p1 = [-radius, 0, 0]    # left edge
    p2 = [radius, 0, 0]     # right edge
    p3 = [0, radius, 0]     # top edge
    p4 = [0, -radius, 0]    # bottom edge
    ball_corners = np.array([p1, p2, p3, p4], dtype=np.float32)
    
    # ==========================================================================
    # TODO: Prepare image points (the detected corner points in pixels)
    # ==========================================================================
    # Hint: Reshape corners array to (4, 2) and ensure float32 type
    # YOUR CODE HERE    
    image_corners = np.float32(np.array(corners).reshape(4, 2))

    

    # ==========================================================================
    # TODO: Construct camera intrinsic matrix
    # ==========================================================================
    # The camera matrix format is:
    #   [[fx,  0, cx],
    #    [ 0, fy, cy],
    #    [ 0,  0,  1]]
    # where fx, fy are focal lengths and cx, cy are principal point coordinates
    # Hint: Access intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy
    # YOUR CODE HERE
    camera_intrinsics = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ], dtype=np.float32)
    
    
    # ==========================================================================
    # TODO: Solve PnP to get rotation and translation
    # ==========================================================================
    # Hint: Use cv2.solvePnP()
    # Returns: (success, rvec, tvec) where rvec is rotation vector, tvec is translation
    # YOUR CODE HERE
    success, rvec, tvec = cv2.solvePnP(ball_corners, image_corners, camera_intrinsics, None)

    # Convert rotation vector to rotation matrix (required return format)
    if success:
        rot_matrix = np.eye(3, 3, dtype=float)  # Create rotation matrix
        rot_matrix *= rvec                      # convert rotation vector to rotation matrix
        return rot_matrix, tvec
    else:
        raise ValueError("No solution")



def detect_balls(image):
    """
    Detect colored balls in the input image using computer vision.
    
    Pipeline:
        1. Convert image to HSV color space for robust color detection
        2. Detect circular shapes using Hough Circle Transform
        3. For each detected circle:
           - Extract the color by analyzing hue values
           - Classify as red, orange, yellow, or blue
           - Record position and radius
    
    Args:
        image: BGR color image from camera
    
    Returns:
        list: List of tuples (color, (cx, cy), radius) for each detected ball
              Returns None if no balls detected
    """
    # ==========================================================================
    # TODO: DETECT CIRCULAR OBJECTS IN IMAGE
    # ==========================================================================
    # Implement your circle detection pipeline here. Consider approaches such as:
    #   - Hough Circle Transform (cv2.HoughCircles)
    #   - Contour detection (cv2.findContours) with circularity filtering
    #   - Color-based segmentation followed by shape analysis
    #   - Edge detection + morphological operations
    #
    # Your implementation should:
    #   - Detect circles reliably under varying lighting conditions
    #   - Handle multiple balls of different colors
    #   - Return circle parameters (center x, y and radius) for each detection
    #
    # Recommended: Store results in a variable called 'circles' with format:
    #              array of shape (1, N, 3) where each circle is [x, y, radius]
    #              (This matches cv2.HoughCircles output format)
    #
    # Explore different preprocessing techniques and parameters!
    # YOUR CODE HERE

    original = image.copy()
    
    # Brightness/Contrast
    image = cv2.convertScaleAbs(image, alpha=1.2, beta=40)

    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define HSV ranges for red and blue (1°-180°)    
    # Red
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([9, 255, 255])
    lower_red2 = np.array([150, 50, 50])
    upper_red2 = np.array([180, 255, 255])  

    # Blue
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([110, 255, 255])

    # Yellow
    lower_yellow = np.array([15, 50, 50])
    upper_yellow = np.array([100, 255, 255])

    # Orange
    lower_orange = np.array([5, 50, 50])
    upper_orange = np.array([25, 255, 255])

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
    min_area = 500  # Adjust based on your image
    min_circularity = 0.5  # Lower threshold for circularity

    result = []  # List to store (color, center, radius) tuples

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

                # Circle
                color = 'red'
                center = [cx, cy]

                # Append to result
                result.append((color, center, radius))

                # Draw the outer circle
                cv2.circle(original, (cx, cy), radius, (0, 255, 0), 2)
                # Draw the center point
                cv2.circle(original, (cx, cy), 3, (0, 0, 255), -1)
                
                # Add measurements
                # cv2.putText(original, f'Area: {area:.0f}', 
                #           (cx-20, cy-40),
                #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # cv2.putText(original, f'Circ: {circularity:.2f}', 
                #           (cx-20, cy-20),
                #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Color: Red', 
                          (cx-20, cy-20),
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
                
                # Circle
                color = 'blue'
                center = [cx, cy]

                # Append to result
                result.append((color, center, radius))

                # Draw the outer circle
                cv2.circle(original, (cx, cy), radius, (0, 255, 0), 2)
                # Draw the center point
                cv2.circle(original, (cx, cy), 3, (0, 0, 255), -1)
                
                # Add measurements
                # cv2.putText(original, f'Area: {area:.0f}', 
                #           (cx-20, cy-40),
                #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # cv2.putText(original, f'Circ: {circularity:.2f}', 
                #           (cx-20, cy-20),
                #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Color: Blue', 
                          (cx-20, cy-20),
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

                # Circle
                color = 'yellow'
                center = [cx, cy]

                # Append to result
                result.append((color, center, radius))
                
                # Draw the outer circle
                cv2.circle(original, (cx, cy), radius, (0, 255, 0), 2)
                # Draw the center point
                cv2.circle(original, (cx, cy), 3, (0, 0, 255), -1)
                
                # Add measurements
                # cv2.putText(original, f'Area: {area:.0f}', 
                #           (cx-20, cy-40),
                #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # cv2.putText(original, f'Circ: {circularity:.2f}', 
                #           (cx-20, cy-20),
                #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Color: Yellow', 
                          (cx-20, cy-20),
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

                # Circle
                color = 'orange'
                center = [cx, cy]

                # Append to result
                result.append((color, center, radius))
                
                # Draw the outer circle
                cv2.circle(original, (cx, cy), radius, (0, 255, 0), 2)
                # Draw the center point
                cv2.circle(original, (cx, cy), 3, (0, 0, 255), -1)
                
                # Add measurements
                # cv2.putText(original, f'Area: {area:.0f}', 
                #           (cx-20, cy-40),
                #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # cv2.putText(original, f'Circ: {circularity:.2f}', 
                #           (cx-20, cy-20),
                #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(original, f'Color: Orange', 
                          (cx-20, cy-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # Terminal Print
                print(f"Orange: {cx}, {cy}")

    # cv2.imshow('original', original)
    # cv2.imshow('image', image)
    # cv2.imshow('red', red_result)
    cv2.imshow('blue', blue_result)
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


    # If no circles detected, show image and return None
    if result is None or len(result[0]) == 0:
        cv2.imshow('Detection', image)
        cv2.waitKey(1)
        return None
    
    # Display detection results
    cv2.imshow('Detection', image)
    cv2.waitKey(1)
    
    return result if result else None


# ============================================================================
# MOTION CONTROL: TRAJECTORY PLANNING AND EXECUTION
# ============================================================================

def move_trajectory(robot, target_pos, traj_time=TRAJECTORY_TIME):
    """
    Move robot to target position using smooth quintic trajectory.
    
    This function:
    1. Gets current robot position
    2. Plans a quintic (5th-order polynomial) trajectory in task space
    3. Converts entire trajectory to joint space using inverse kinematics
    4. Executes trajectory with precise timing
    
    Args:
        robot: Robot instance
        target_pos: Target position [x, y, z, pitch] in mm and degrees
        traj_time: Duration of trajectory in seconds
    """
    # ==========================================================================
    # TODO: Get current joint positions and end-effector position
    # ==========================================================================
    # YOUR CODE HERE
    current_joints = robot.get_joints_readings()[0]
    current_ee_pos = robot.get_ee_pos(current_joints)[:4]
    
    
    # ==========================================================================
    # TODO: Create task-space trajectory from current to target position
    # ==========================================================================
    # Hint: Stack current_pos and target_pos, create TrajPlanner, call get_quintic_traj()
    # YOUR CODE HERE
    points_num = 998  # Number of intermediate waypoints per segment
    ee_poses = np.array([current_ee_pos, target_pos], dtype=np.float32)
    tj = TrajPlanner(ee_poses)
    trajectories = tj.get_quintic_traj(traj_time, points_num)
    
    
    # ==========================================================================
    # TODO: Convert entire trajectory to joint space
    # ==========================================================================
    # YOUR CODE HERE
    joint_angles = np.copy(trajectories)
    # Find IK for Task Space Trajectory
    print("Computing IK for trajectory...")
    try:
        for points in range(len(trajectories)):
            joint_angles[points,1:] = np.array([robot.get_ik(trajectories[points, 1:])])
    except ValueError as e:
        raise ValueError(f"End-Effector Pose Unreachable: {e}")
    
    
    # TODO: Calculate time step between waypoints
    dt = trajectories[1, 0] - trajectories[0, 0]
    robot.write_time(dt)
    
    # ==========================================================================
    # TODO: Execute trajectory with precise timing (see Lab 4)
    # ==========================================================================
    # Hint: Record start time, then for each waypoint:
    #   - Calculate target time for this waypoint
    #   - Wait until that time
    #   - Send joint commands
    # This ensures smooth, consistent motion regardless of computation time
    # YOUR CODE HERE
    print("\nExecuting trajectory...")
    start_time = time.perf_counter()
    # Execute trajectory by streaming commands
    for i in range(1, len(joint_angles)):
        # Calculate when this command should be sent
        target_time = start_time + joint_angles[i, 0]
        
        # Wait until it's time to send this command
        while time.perf_counter() < target_time:            
            # Small sleep to prevent CPU overload
            time.sleep(0.001)  # 1ms sleep
        
        # Send the command at the scheduled time
        robot.write_joints(joint_angles[i, 1:])


# ============================================================================
# PICK AND PLACE OPERATIONS
# ============================================================================

def pick_ball(robot, ball_pos):
    """
    Execute a pick operation to grasp a ball.
    
    Sequence:
        1. Open gripper
        2. Move to approach position (above ball)
        3. Move down to grasp position
        4. Close gripper
        5. Lift ball to clear workspace
    
    Args:
        robot: Robot instance
        ball_pos: Ball position [x, y, z] in robot frame (mm)
    """
    print(f"Picking ball at {ball_pos}")
    
    # ==========================================================================
    # Open gripper
    # ==========================================================================
    # Use robot.write_gripper(1) for open, wait for motion
    robot.write_gripper(1)
    time.sleep(0.5)
    
    # ==========================================================================
    # Move to approach position (above the ball)
    # ==========================================================================
    # Create position [x, y, z_high, pitch] where z_high is ~100mm
    # Use steep pitch angle (e.g., -80°) for vertical approach
    approach = np.array([ball_pos[0], ball_pos[1], 100, -80], dtype=np.float32)   # may need tuning
    move_trajectory(robot, approach, TRAJECTORY_TIME)
    
    # ==========================================================================
    # Move down to grasp position
    # ==========================================================================
    # Lower z to just above table surface (e.g., 39mm for 30mm ball radius)
    # Adjust this height based on your table height and ball size!
    grasp = np.array([ball_pos[0], ball_pos[1], 39, -80], dtype=np.float32)      # may need tuning
    move_trajectory(robot, grasp, TRAJECTORY_TIME * 0.5)
    
    # ==========================================================================
    # Close gripper to grasp ball
    # ==========================================================================
    #  Use robot.write_gripper(0) for close, wait for secure grasp
    robot.write_gripper(0)
    time.sleep(1)
    
    # ==========================================================================
    # Lift ball to clear workspace
    # ==========================================================================
    # Move back up to approach height
    lift = [ball_pos[0], ball_pos[1], 100, -80] # may need tuning
    move_trajectory(robot, lift, TRAJECTORY_TIME * 0.5)


def place_ball(robot, color):
    """
    Place ball in the appropriate color-coded bin.
    
    Args:
        robot: Robot instance
        color: Ball color string ('red', 'orange', 'yellow', 'blue')
    """
    print(f"Placing {color} ball")
    
    # ==========================================================================
    # Get bin position for this color
    # ==========================================================================
    # Look up position in BINS dictionary
    bin_pos = BINS[color]
    
    # ==========================================================================
    # Move to bin location
    # ==========================================================================
    move_trajectory(robot, bin_pos, TRAJECTORY_TIME)
    
    # ==========================================================================
    # Release ball by opening gripper
    # ==========================================================================
    robot.write_gripper(1)
    time.sleep(1)


def go_home(robot):
    """
    Return robot to home position for next detection cycle.
    
    Args:
        robot: Robot instance
    """
    move_trajectory(robot, HOME_POSITION, TRAJECTORY_TIME)


# ============================================================================
# MAIN CONTROL LOOP
# ============================================================================

def main():
    """
    Main control loop for the robotic sorting system.
    
    Workflow:
        1. Initialize robot, camera, and calibration
        2. Move to home position
        3. Loop:
           a. Capture image and detect balls
           b. Convert detected positions to robot frame
           c. Filter balls within workspace
           d. Pick and place first ball
           e. Repeat
    """
    print("="*60)
    print("Lab 8: Robotic Sorting System")
    print("="*60)
    
    # ==========================================================================
    # INITIALIZATION
    # ==========================================================================
    
    # TODO: Initialize robot, camera, and get intrinsics
    # Hint: Create Robot(), Realsense(), and get intrinsics
    # YOUR CODE HERE
    robot = Robot()
    camera = Realsense()
    intrinsics = camera.get_intrinsics()
    
    
    # ==========================================================================
    # TODO: Load camera-robot calibration matrix
    # ==========================================================================
    # Hint: Use np.load() to load 'camera_robot_transform.npy'
    # This matrix transforms points from camera frame to robot frame
    # YOUR CODE HERE
    try:
        T_cam_to_robot = np.load('camera_robot_transform.npy')  # Replace with loaded matrix
        print("Calibration loaded successfully")
    except FileNotFoundError:
        print("Error: camera_robot_transform.npy not found!")
        print("Please run lab6_2.py first to calibrate.")
        return
    
    
    
    # ==========================================================================
    # TODO: Move to home position
    # ==========================================================================
    # Hint: Use inverse kinematics to find joint angles, then command them
    # YOUR CODE HERE
    # =========================================================================
    # MOVE TO START POSITION
    # =========================================================================
    print("\nMoving to start position...")
    
    # Set position mode and trajectory time
    robot.write_mode("position")
    traj_time = 3.0
    robot.write_time(traj_time)
    
    # Start position: [x, y, z, gripper_angle] in mm and degrees
    start_position = [100, 0, 220, -15]
    start_joints = robot.get_ik(start_position)  # Replace with IK solution

    # TODO: Move to start position: write joints and wait for motion to complete
    # Hint: Use robot.get_ik(), robot.write_joints(), time.sleep()
    # YOUR CODE HERE
    # TODO: Enable motors
    traj_time = 5   # 5s write time
    robot.write_motor_state(True)
    robot.write_mode('position')
    robot.write_time(traj_time)
    
    robot.write_joints(start_joints)
    time.sleep(traj_time)
    
    print("Robot ready for visual servoing")
    
    
    # ==========================================================================
    # TODO: Open gripper initially
    # ==========================================================================
    # YOUR CODE HERE
    robot.write_gripper(True)
    
    print(f"\nReady! Using TRAJECTORY control")
    print("Press Ctrl+C to stop\n")
    
    # ==========================================================================
    # MAIN CONTROL LOOP
    # ==========================================================================
    
    try:
        iteration = 0
        
        while True:
            print(f"\n{'='*60}")
            print(f"Iteration {iteration}")
            
            # ==================================================================
            # STEP 1: CAPTURE IMAGE AND DETECT BALLS
            # ==================================================================
            
            # TODO: Get camera frame
            # Hint: Use camera.get_frames() which returns (color, depth)
            # YOUR CODE HERE
            color_frame, _ = camera.get_frames()
            if color_frame is None:
                continue            
            
            # Capture image for tuning
            # cv2.imwrite("output.png", color_frame)

            
            # TODO: Detect balls in image
            # Hint: Call detect_balls() function
            # YOUR CODE HERE
            spheres = detect_balls(color_frame)
            
            # Check if any balls detected
            if spheres is None:
                print("No balls detected")
                time.sleep(1)
                iteration += 1
                continue
            
            print(f"Detected {len(spheres)} ball(s)")
            
            # ==================================================================
            # STEP 2: CONVERT DETECTIONS TO ROBOT FRAME
            # ==================================================================
            
            robot_spheres = []  # List to store (color, robot_position) tuples
            
            for color, (cx, cy), radius in spheres:
                
                # ==============================================================
                # TODO: Create corner points for PnP algorithm
                # ==============================================================
                # Hint: Create 4 points at [left, right, bottom, top] of circle
                # Format: [[cx - radius, cy], [cx + radius, cy], ...]
                # YOUR CODE HERE
                spheres_corners = np.array([[cx - radius, cy], [cx + radius, cy], [cx, cy - radius], [cx, cy + radius]], dtype=np.float32)
                
                
                # ==============================================================
                # TODO: Get 3D position in camera frame using PnP
                # ==============================================================
                # Hint: Call get_ball_pose() with corners, intrinsics, and BALL_RADIUS
                # Returns (rotation, translation) - we only need translation
                # YOUR CODE HERE     
                _ , trans_vector = get_ball_pose(spheres_corners, intrinsics, radius)           
                
                # ==============================================================
                # TODO: Transform position to robot frame
                # ==============================================================
                # Hint: 
                #   1. Flatten cam_pos and append 1 for homogeneous coordinates
                #   2. Multiply by transformation matrix: T_cam_to_robot @ pos_hom
                #   3. Extract first 3 elements for 3D position
                # YOUR CODE HERE
                tag_pos_camera = trans_vector.flatten()
                tag_pos_camera_hom = np.append(tag_pos_camera, 1)
                robot_pos = T_cam_to_robot @ tag_pos_camera_hom
                
                
                # ==============================================================
                # Check if position is within workspace bounds
                # ==============================================================
                # Check if X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX
                # Skip balls outside workspace for safety
                if not (X_MIN <= robot_pos[0] <= X_MAX and 
                        Y_MIN <= robot_pos[1] <= Y_MAX):
                    print(f"  Skipping {color} ball outside workspace: {robot_pos}")
                    continue
                
                
                robot_spheres.append((color, robot_pos))
                print(f"  {color}: {robot_pos}")
            
            # Check if any valid balls found
            if not robot_spheres:
                print("No balls in workspace")
                time.sleep(1)
                iteration += 1
                continue
            
            # ==================================================================
            # STEP 3: PICK AND PLACE FIRST BALL
            # ==================================================================
            
            # Get first ball from list
            color, pos = robot_spheres[0]
            
            print(f"\nSorting {color} ball at {pos}")
            
            # TODO: Execute pick-and-place sequence
            # Hint: Call pick_ball(), place_ball(), and go_home()
            # YOUR CODE HERE
            pick_ball(robot, pos)
            time.sleep(0.5)
            place_ball(robot, color)
            time.sleep(0.5)
            go_home(robot)
            time.sleep(0.5)
            
            
            iteration += 1
            time.sleep(1)  # Brief pause before next cycle
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        # TODO: stop the camera
        
        cv2.destroyAllWindows()
        print("Done!")


# ============================================================================
# PROGRAM ENTRY POINT
# ============================================================================

def test():
    script_dir = Path(__file__).parent
    image_path = script_dir / "inclasstest.png"   # Get file path
    # Read Image
    image_path = Path(image_path)
    image = cv2.imread(str(image_path))
    # cv2.imshow('testing', image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


    spheres = detect_balls(image)
    # Get first ball from list
    for ball in range(len(spheres)):
        color, pos, radius = spheres[ball]
        print(f"\nSorting {color} ball at {pos} with radius {radius}")
    



if __name__ == "__main__":
    # main()
    test()