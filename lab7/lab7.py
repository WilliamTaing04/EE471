"""
(c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
Lab 7 Starter Code: Position-Based Visual Servoing with PID Control
Implements real-time visual servoing to track an AprilTag target.
"""

import numpy as np
import cv2
import time
import pickle
from pathlib import Path

from classes.Robot import Robot
from classes.Realsense import Realsense
from classes.AprilTags import AprilTags

# Directory where this file lives
HERE = Path(__file__).parent

def save_to_pickle(data: dict, filename: str):
    """Save data dictionary to a pickle file in the same folder as this script."""
    path = HERE / filename
    with open(path, "wb") as f:
        pickle.dump(data, f)

def load_from_pickle(filename: str):
    """Load data dictionary from a pickle file in the same folder as this script."""
    path = HERE / filename
    with open(path, "rb") as f:
        return pickle.load(f)
    

class PIDController:
    def __init__(self, dim=3, dt=0.05):
        # Initialize gains (tuned for position control in mm)
        self.Kp = 1.0 * np.eye(dim) # Proportional gain
        self.Ki = 0.1 * np.eye(dim) # Integral gain
        self.Kd = 0.1 * np.eye(dim) # Derivative gain
        # Initialize error terms
        self.error_integral = np.zeros(dim)
        self.error_prev = np.zeros(dim)
        self.dt = dt # Control period in seconds

    def compute_pid(self, error):
        # Initialize u
        u = np.zeros(3)

        # Accumulate Error
        self.error_integral += error * self.dt

        # Calculate u
        u = (self.Kp @ error) + (self.Ki @ self.error_integral) + (self.Kd @ ((error - self.error_prev) / self.dt))

        # Save error for next dt
        self.error_prev = error

        return u


def main():
    """
    Main visual servoing control loop.
    """
    print("="*60)
    print("Lab 7: Position-Based Visual Servoing")
    print("="*60)
    
    # =========================================================================
    # INITIALIZATION
    # =========================================================================
    print("\nInitializing system...")
    
    # TODO: Initialize robot, camera, and AprilTag detector
    # Hint: Create Robot(), Realsense(), and AprilTags() instances
    # YOUR CODE HERE
    robot = Robot()      # Replace with Robot()
    camera = Realsense()     # Replace with Realsense()
    detector = AprilTags()   # Replace with AprilTags()
    
    
    # TODO: Get camera intrinsics
    # Hint: Use camera.get_intrinsics()
    # YOUR CODE HERE
    intrinsics = camera.get_intrinsics()  # Replace with actual intrinsics
    
    
    # TODO: Define control timestep (CRITICAL - must match PID dt!)
    # Hint: 0.05 seconds = 20 Hz, 0.02 seconds = 50 Hz
    # YOUR CODE HERE
    dt = 0.05  # Control loop period in seconds
    # max_vel = 50  # Max mm/s
    
    # Pre-allocate arrays for data collection (over-allocate for safety)
    max_samples = 2400     # 20Hz => 20(samples/s)*120s = 2400samples
    data_time = np.zeros(max_samples)                 # Time (s)
    data_ee_pos = np.zeros((max_samples, 3))          # End-effector pose [x,y,z] (mm)
    data_ee_pos_des = np.zeros((max_samples, 3))      # Desired End-effector pose [x,y,z] (mm)
    data_error = np.zeros((max_samples, 3))           # End-effector Error [ex,ey,ez] (mm)
    data_control_output = np.zeros((max_samples, 3))  # PID output [vx,vy,vz] (mm/s)
    data_q_dot = np.zeros((max_samples, 4))           # Joint velocities (rad/s)
    data_tag = np.zeros(max_samples)                  # Boolean tag visible
    count = 0  # Sample counter
    
    # TODO: Set AprilTag physical size in millimeters
    # Hint: Measure your actual tag!
    # YOUR CODE HERE
    TAG_SIZE = 40.0  # Update this value
    
    
    # TODO: Initialize PID controller with SAME dt as control loop
    # Hint: pid = PIDController(dim=3, dt=dt)
    # YOUR CODE HERE
    pid = PIDController(dim=3, dt=dt)  # Replace with PIDController instance
    
    
    # TODO: Load camera-robot calibration matrix
    # Hint: Use np.load('camera_robot_transform.npy')
    # YOUR CODE HERE    
    try:
        T_cam_to_robot = np.load('camera_robot_transform.npy')  # Replace with loaded matrix
        print("Calibration loaded successfully")
    except FileNotFoundError:
        print("Error: camera_robot_transform.npy not found!")
        print("Please run lab6_2.py first to calibrate.")
        return
    
    
    # TODO: Define desired offset from tag (mm)
    # Hint: This defines where end-effector should be relative to tag
    # Example: [0, 0, 50] means 50mm above tag
    # YOUR CODE HERE
    target_offset = np.array([-50, 0, 50])  # Adjust as needed
    print(f"Target offset from tag: {target_offset} mm")
    
    
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

    
    # TODO: Switch robot to velocity control mode
    # Hint: Use robot.write_mode("velocity")
    # YOUR CODE HERE
    robot.write_mode("velocity")
    
    print("Robot ready for visual servoing")
    
    
    # =========================================================================
    # MAIN CONTROL LOOP
    # =========================================================================
    print("\nStarting visual servoing control loop...")
    print("Press 'q' to quit\n")
    
    iteration = 0
    begining_time = time.time()  # Record start time for data colleciton

    try:
        while True:
            # TODO: Record start time for fixed timestep enforcement
            # Hint: start_time = time.time()
            # YOUR CODE HERE
            start_time = time.time()  # Replace with actual time
            
            # -----------------------------------------------------------------
            # STEP 1: CAPTURE FRAME AND DETECT TAG
            # -----------------------------------------------------------------
            
            # Get camera frame
            color_frame, _ = camera.get_frames()
            if color_frame is None:
                continue

            # TODO: Detect AprilTags in frame
            # Hint: Use detector.detect_tags(color_frame)
            # YOUR CODE HERE
            tags = detector.detect_tags(color_frame)  # Replace with detected tags
            
            
            # Check if any tags detected
            if len(tags) > 0:
                tag = tags[0]  # Use first detected tag
                
                # Draw tag detection on image for visualization
                color_frame = detector.draw_tags(color_frame, tag)
                
                # -----------------------------------------------------------------
                # STEP 2: GET TAG POSE IN CAMERA FRAME
                # -----------------------------------------------------------------
                
                # TODO: Get tag pose using PnP
                # Hint: detector.get_tag_pose(tag.corners, intrinsics, TAG_SIZE)
                # Returns: (rotation_matrix, translation_vector)
                # YOUR CODE HERE
                rot_matrix, trans_vector = detector.get_tag_pose(tag.corners, intrinsics, TAG_SIZE)   # Replace with actual rotation and translation
                
                
                # TODO: Extract tag position in camera frame (already in mm)
                # Hint: Flatten trans_vector to get 1D array
                # YOUR CODE HERE
                tag_pos_camera = trans_vector.flatten()  # Replace with position array (shape: 3,)
                
                
                # -----------------------------------------------------------------
                # STEP 3: TRANSFORM TO ROBOT FRAME
                # -----------------------------------------------------------------
                
                # TODO: Convert to homogeneous coordinates
                # Hint: Append 1 to make [x, y, z, 1]
                # YOUR CODE HERE
                tag_pos_camera_hom = np.append(tag_pos_camera, 1)  # Replace with homogeneous coordinates
                
                
                # TODO: Apply camera-to-robot transformation
                # YOUR CODE HERE
                tag_pos_robot_hom = T_cam_to_robot @ tag_pos_camera_hom  # Replace with transformed coordinates
                
                
                # TODO: Extract 3D position from homogeneous coordinates
                # Hint: Take first 3 elements
                # YOUR CODE HERE
                tag_pos_robot = tag_pos_robot_hom[:3]  # Replace with 3D position (shape: 3,)
                
                
                # -----------------------------------------------------------------
                # STEP 4: CALCULATE DESIRED END-EFFECTOR POSITION
                # -----------------------------------------------------------------
                
                # TODO: Add offset to tag position to get desired EE position
                # Hint: desired_ee_pos = tag_pos_robot + target_offset
                # YOUR CODE HERE
                desired_ee_pos = tag_pos_robot + target_offset  # Replace with desired position
                
                
                # -----------------------------------------------------------------
                # STEP 5: GET CURRENT END-EFFECTOR POSITION
                # -----------------------------------------------------------------
                
                # TODO: Get current joint positions
                # YOUR CODE HERE
                current_joints = robot.get_joints_readings()[0]  # Replace with joint readings
                
                
                # TODO: Get current end-effector position using forward kinematics
                # Hint: robot.get_ee_pos(current_joints) returns [x, y, z, ...]
                # Take only first 3 elements (position)
                # YOUR CODE HERE
                current_ee_pos = robot.get_ee_pos(current_joints)[:3]  # Replace with current position (shape: 3,)
                
                
                # -----------------------------------------------------------------
                # STEP 6: CALCULATE POSITION ERROR
                # -----------------------------------------------------------------
                
                # TODO: Compute position error
                # Hint: error = desired_position - current_position
                # YOUR CODE HERE
                error = desired_ee_pos - current_ee_pos  # Replace with error vector
                
                
                # -----------------------------------------------------------------
                # STEP 7: COMPUTE PID CONTROL OUTPUT
                # -----------------------------------------------------------------
                
                # TODO: Use PID controller to compute velocity command
                # Hint: Call pid.compute()
                # YOUR CODE HERE
                velocity_cmd = pid.compute_pid(error)  # Replace with PID output (mm/s)

                # Limit max velocity
                # velocity_cmd[velocity_cmd > max_vel] = max_vel                
                
                # -----------------------------------------------------------------
                # STEP 8: CONVERT TO JOINT VELOCITIES
                # -----------------------------------------------------------------
                
                # TODO: Get robot Jacobian at current configuration
                # Hint: robot.get_jacobian()
                # YOUR CODE HERE
                J = robot.get_jacobian(current_joints)  # Replace with Jacobian matrix
                
                
                # TODO: Extract position part of Jacobian (first 3 rows)
                # Hint: J_linear = J[:3, :]
                # YOUR CODE HERE
                J_linear = J[:3, :]  # Replace with position Jacobian
                
                
                # TODO: Compute joint velocities using pseudo-inverse
                # Hint: joint_vel = pinv(J_linear) @ velocity_cmd
                # Use np.linalg.pinv()
                # YOUR CODE HERE
                joint_vel = np.rad2deg(np.linalg.pinv(J_linear) @ velocity_cmd)  # Replace with joint velocities
                
                
                # -----------------------------------------------------------------
                # STEP 9: COMMAND ROBOT
                # -----------------------------------------------------------------
                
                # TODO: Send joint velocities to robot
                # Hint: robot.write_velocities()
                # OpenManipulator-X has 4 joints (excluding gripper)
                # YOUR CODE HERE
                robot.write_velocities(joint_vel)
                
                
                # -----------------------------------------------------------------
                # STEP 10: DISPLAY STATUS
                # -----------------------------------------------------------------
                
                # Print status every 40 iterations (~2 seconds at 20Hz)
                if iteration % 40 == 0:
                    print(f"\nIteration: {iteration}")
                    print(f"Tag position (robot): {tag_pos_robot}")
                    print(f"Current EE position:  {current_ee_pos}")
                    print(f"Desired EE position:  {desired_ee_pos}")
                    print(f"Error: {error} mm")
                    print(f"Error magnitude: {np.linalg.norm(error):.2f} mm")

                # Collect Data-----------------------------------------------------
                if count < max_samples:
                    data_time[count] = time.perf_counter() - begining_time  # Time stamps (s)
                    data_ee_pos[count, :] = current_ee_pos                  # End-effector pose [x,y,z] (mm)
                    data_ee_pos_des[count, :] = desired_ee_pos              # Desired End-effector pose [x,y,z] (mm)
                    data_error[count, :] = error                            # End-effector Error [ex,ey,ez] (mm)
                    data_control_output[count, :] = velocity_cmd            # PID output [vx,vy,vz] (mm/s)
                    data_q_dot[count, :] = joint_vel                        # Joint velocities (rad/s)
                    count += 1
                
            else:
                # -----------------------------------------------------------------
                # NO TAG DETECTED - STOP ROBOT
                # -----------------------------------------------------------------
                
                # TODO: Stop robot motion by sending zero velocities
                # Hint: robot.write_velocities([0, 0, 0, 0])
                # YOUR CODE HERE
                robot.write_velocities([0,0,0,0])
                
                
                if iteration % 40 == 0:
                    print("\nNo AprilTag detected - robot stopped")
            
            
            # -----------------------------------------------------------------
            # DISPLAY AND USER INTERACTION
            # -----------------------------------------------------------------
            
            # Display camera image
            cv2.imshow('Visual Servoing', color_frame)
            key = cv2.waitKey(1)
            
            # Check for quit key press ('q' or ESC)
            if key & 0xFF == ord('q') or key == 27:
                print("\nQuitting...")
                break
            
            iteration += 1
            

            # -----------------------------------------------------------------
            # MAINTAIN FIXED TIMESTEP (CRITICAL!)
            # -----------------------------------------------------------------
            
            # Enforce consistent loop timing
            elapsed = time.time() - start_time
            if elapsed < dt:
                time.sleep(dt - elapsed)
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # =====================================================================
        # CLEANUP
        # =====================================================================
        print("\nStopping robot and cleaning up...")
        robot.write_velocities([0, 0, 0, 0])
        camera.stop()
        cv2.destroyAllWindows()
        total_time = time.perf_counter() - begining_time
        print(f"\nTotal execution time: {total_time:.2f} s")
        print("Done!")

    # Trim unused portions of pre-allocated arrays
    data_time = data_time[:count]
    data_ee_pos = data_ee_pos[:count, :]
    data_ee_pos_des = data_ee_pos_des[:count, :]
    data_error = data_error[:count, :]
    data_control_output = data_control_output[:count, :]
    data_q_dot = data_q_dot[:count, :]
    data_tag = data_tag[:count]

    # TODO: Save all data to pickle file
    filename='lab7_data_test.pkl'
    # Create dictionary with all collected data and control parameters
    print(f"\nSaving data to {filename}...")
    data_dict = {
    'time': data_time, # Timestamp (s)
    'pos_current': data_ee_pos, # Current EE position [x,y,z] (mm)
    'pos_desired': data_ee_pos_des, # Desired EE position [x,y,z] (mm)
    'error': data_error, # Position error [ex,ey,ez] (mm)
    'control_output': data_control_output, # PID output [vx,vy,vz] (mm/s)
    'joint_vel': data_q_dot, # Joint velocities (rad/s)
    'tag_detected': data_tag # Boolean: tag visible
    }

    # TODO: Write dictionary to pickle file
    save_to_pickle(data_dict, filename)
    print("Data saved successfully!")

def plots():
    # Load data from pickle files
    data_xstep = load_from_pickle("lab7_data_Xstep.pkl")
    data_ystep = load_from_pickle("lab7_data_Ystep.pkl")
    data_zstep = load_from_pickle("lab7_data_Zstep.pkl")
    data_rectangle = load_from_pickle("lab7_data_rectangle.pkl")

    # X step data
    time_pkl_xstep = data_xstep["time"]
    ee_pose_pkl_xstep = data_xstep["pos_current"]
    ee_pose_des_pkl_xstep = data_xstep["pos_desired"]
    error_pkl_xstep = data_xstep["error"]
    control_output_pkl_xstep = data_xstep["control_output"]
    joint_vel_pkl_xstep = data_xstep["joint_vel"]

    # Y step data
    time_pkl_ystep = data_ystep["time"]
    ee_pose_pkl_ystep = data_ystep["pos_current"]
    ee_pose_des_pkl_ystep = data_ystep["pos_desired"]
    error_pkl_ystep = data_ystep["error"]
    control_output_pkl_ystep = data_ystep["control_output"]
    joint_vel_pkl_ystep = data_ystep["joint_vel"]

    # Z step data
    time_pkl_zstep = data_zstep["time"]
    ee_pose_pkl_zstep = data_zstep["pos_current"]
    ee_pose_des_pkl_zstep = data_zstep["pos_desired"]
    error_pkl_zstep = data_zstep["error"]
    control_output_pkl_zstep = data_zstep["control_output"]
    joint_vel_pkl_zstep = data_zstep["joint_vel"]

    # Rectangle data
    time_pkl_rect = data_rectangle["time"]
    ee_pose_pkl_rect = data_rectangle["pos_current"]
    ee_pose_des_pkl_rect = data_rectangle["pos_desired"]
    error_pkl_rect = data_rectangle["error"]
    control_output_pkl_rect = data_rectangle["control_output"]
    joint_vel_pkl_rect = data_rectangle["joint_vel"]

    # Creating new time lists(recorded times are broken)
    new_time_pkl_xstep = np.arange(0, time_pkl_xstep.size*0.05, 0.05)
    new_time_pkl_ystep = np.arange(0, time_pkl_ystep.size*0.05, 0.05)
    new_time_pkl_zstep = np.arange(0, time_pkl_zstep.size*0.05, 0.05)
    new_time_pkl_rect = np.arange(0, time_pkl_rect.size*0.05, 0.05)

    np.set_printoptions(precision=3, suppress=1)    # set print precision and suppression
    # print(error_pkl_xstep.shape)
    # print(np.sqrt(np.mean(error_pkl_xstep[50:]**2)))

    # EE pose actual vs desired
    Robot.plot_ee_pose_desvsacc_list(Robot, new_time_pkl_xstep, ee_pose_pkl_xstep, ee_pose_des_pkl_xstep, "Xstep Desired vs Actual Position")
    Robot.plot_ee_pose_desvsacc_list(Robot, new_time_pkl_ystep, ee_pose_pkl_ystep, ee_pose_des_pkl_ystep, "Ystep Desired vs Actual Position")
    Robot.plot_ee_pose_desvsacc_list(Robot, new_time_pkl_zstep, ee_pose_pkl_zstep, ee_pose_des_pkl_zstep, "Zstep Desired vs Actual Position")
    Robot.plot_ee_pose_desvsacc_list(Robot, new_time_pkl_rect, ee_pose_pkl_rect, ee_pose_des_pkl_rect, "Rectangle Desired vs Actual Position")

    # EE error vs time
    Robot.plot_ee_pose_error_list(Robot, new_time_pkl_xstep, error_pkl_xstep, 5, "Xstep Error vs Time (5mm SSE)")
    Robot.plot_ee_pose_error_list(Robot, new_time_pkl_ystep, error_pkl_ystep, 5, "Ystep Error vs Time (5mm SSE)")
    Robot.plot_ee_pose_error_list(Robot, new_time_pkl_zstep, error_pkl_zstep, 5, "Zstep Error vs Time (5mm SSE)")
    Robot.plot_ee_pose_error_list(Robot, new_time_pkl_rect, error_pkl_rect, 5, "Rectangle Error vs Time (5mm SSE)")

    # Control Output vs time
    Robot.plot_control_output_list(Robot, new_time_pkl_xstep, control_output_pkl_xstep, "Xstep Control Output vs Time")
    Robot.plot_control_output_list(Robot, new_time_pkl_ystep, control_output_pkl_ystep, "Ystep Control Output vs Time")
    Robot.plot_control_output_list(Robot, new_time_pkl_zstep, control_output_pkl_zstep, "Zstep Control Output vs Time")
    Robot.plot_control_output_list(Robot, new_time_pkl_rect, control_output_pkl_rect, "Rectangle Control Output vs Time")

    # Joint Velocities
    Robot.plot_angle_velocity_list(new_time_pkl_xstep, joint_vel_pkl_xstep, "Xstep Joint Velocities vs Time")
    Robot.plot_angle_velocity_list(new_time_pkl_ystep, joint_vel_pkl_ystep, "Ystep Joint Velocities vs Time")
    Robot.plot_angle_velocity_list(new_time_pkl_zstep, joint_vel_pkl_zstep, "Zstep Joint Velocities vs Time")
    Robot.plot_angle_velocity_list(new_time_pkl_rect, joint_vel_pkl_rect, "Rectangle Joint Velocities vs Time")

    # 3D Trajectory
    Robot.plot_3D_trajectory_desvsacc_list(Robot, ee_pose_pkl_xstep, ee_pose_des_pkl_xstep, 50, "Xstep 3D Trajectory Actual vs Desired")
    Robot.plot_3D_trajectory_desvsacc_list(Robot, ee_pose_pkl_ystep, ee_pose_des_pkl_ystep, 50, "Ystep 3D Trajectory Actual vs Desired")
    Robot.plot_3D_trajectory_desvsacc_list(Robot, ee_pose_pkl_zstep, ee_pose_des_pkl_zstep, 50, "Zstep 3D Trajectory Actual vs Desired")
    Robot.plot_3D_trajectory_desvsacc_list(Robot, ee_pose_pkl_rect, ee_pose_des_pkl_rect, 50, "Rectangle 3D Trajectory Actual vs Desired")



if __name__ == "__main__":
    # main()

    plots()
    