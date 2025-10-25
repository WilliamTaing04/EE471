# EE471 Lab 3_3 10/14/2025
# William Taing wtaing@calpoly.edu
# Description: for lab3 section3 testing: Interface and validate your IK solution with the robot

import time
import pickle
import numpy as np
import matplotlib.pyplot as plt
from classes.Robot import Robot

def save_to_pickle(data: dict, filename: str):
    with open(filename, "wb") as f:
        pickle.dump(data, f)

def load_from_pickle(filename: str):
    with open(filename, "rb") as f:
        return pickle.load(f)

def main():
    traj_time = 5.0                  # sec
    home = [0, 0, 0, 0]              # deg
    poll_dt   = 0.1                  # sec

    # Waypoints for triangle tracing
    wp1 = [25, -100, 150, -60]  # (mm)(deg)
    wp2 = [150, 80, 300, 0]     # (mm)(deg)
    wp3 = [250, -115, 75, -45]  # (mm)(deg)
    wp4 = [25, -100, 150, -60]  # (mm)(deg)
    

    # create list of poses
    pose_list = []  
    pose_list.append(wp1)   
    pose_list.append(wp2)
    pose_list.append(wp3)
    pose_list.append(wp4)

    time_list = []          # time list
    angles_list = []        # motor angles list
    ee_list = []            # end effector position list

    robot = Robot()

    np.set_printoptions(precision=3, suppress=1)    # set print precision and suppression

    # Enable torque and set time-based profile
    robot.write_motor_state(True)
    robot.write_time(traj_time)
    
    # wp1
    print("Homing to wp1 ...")
    robot.write_joints(robot.get_ik(wp1))
    time.sleep(traj_time)

    # Move to target poses
    t_init = time.perf_counter()
    for wp in pose_list:  # for all poses in pose_list
        print(f"\nMoving to {wp} (mm)(deg) over {traj_time:.1f}s ...")
        robot.write_joints(robot.get_ik(wp))    # move robot to pose
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < traj_time: # while moving to pose
            time_list.append(time.perf_counter()-t_init)            # save current time to list
            angles_list.append(robot.get_joints_readings()[0])  # save current joint angle to list
            ee_list.append(robot.get_ee_pos(robot.get_joints_readings()[0]))    # save current ee pose to list
            time.sleep(poll_dt)
    
    
    print("\nMoved to target pose.")    # Finished movement messages

    # Save data to pickle file
    save_to_pickle({"time": time_list, "angles": angles_list, "target_angles": pose_list, "traj_time":traj_time, "ee_pose": ee_list}, "lab3_3_data.pkl")

    # Load data from pickle file
    data = load_from_pickle("lab3_3_data.pkl")
    time_pkl = data["time"]
    angles_pkl = data["angles"]
    ee_pose_pkl = data["ee_pose"]
    
    # End-effector pose vs time
    robot.plot_ee_pose_list(time_pkl, ee_pose_pkl,"End-effector pose vs time")

    # 3D trajectory
    robot.plot_3D_trajectory_list(ee_pose_pkl, "3D Trajectory")

    # Joint angles
    robot.plot_angle_list(time_pkl, angles_pkl,"Motor Angle vs Time")

    # Shutdown (ALWAYS LAST)
    robot.close()

if __name__ == "__main__":
    main()
