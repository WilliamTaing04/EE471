# EE471 Lab 2_4 10/8/2025
# William Taing wtaing@calpoly.edu
# Description: for lab2 section 4 testing(Moves the robot through the waypoint sequence, continuously record time, joint angles, ee position)

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
    wp1 = [0, -45, 60, 50]       # deg
    wp2 = [0, 10, 50, -45]       # deg
    wp3 = [0, 10, 0, -80]        # deg
    wp4 = [0, -45, 60, 50]       # deg

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

    # Home
    print("Homing to [0, 0, 0, 0] deg ...")
    robot.write_joints([0, 0, 0, 0])
    time.sleep(traj_time)

    # Move to target poses
    for pose in pose_list:  # for all poses in pose_list
        print(f"\nMoving to {pose} (deg) over {traj_time:.1f}s ...")
        robot.write_joints(pose)    # move robot to pose
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < traj_time: # while moving to pose
            time_list.append(time.perf_counter()-t0)            # save current time to list
            angles_list.append(robot.get_joints_readings()[0])  # save current joint angle to list
            ee_list.append(robot.get_ee_pos(robot.get_joints_readings()[0]))    # save current ee pose to list
    
    
    print("\nMoved to target pose.")    # Finished movement messages

    # Save data to pickle file
    save_to_pickle({"time": time_list, "angles": angles_list, "target_angles": pose_list, "traj_time":traj_time, "ee_pose": ee_list}, "lab2_4_data.pkl")

    # Load data from pickle file
    data = load_from_pickle("lab2_4_data.pkl")

    # Shutdown (ALWAYS LAST)
    robot.close()

if __name__ == "__main__":
    main()
