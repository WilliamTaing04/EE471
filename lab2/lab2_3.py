# EE471 Lab 2_3 10/8/2025
# William Taing wtaing@calpoly.edu
# Description: for lab2 section 4 testing(Continuously prints the “end-effector to base transformation matrix” as well as the “task space pose)

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
    pose1 = [15, -45, -60, 90]       # deg
    pose2 = [-90, 15, 30, -45]       # deg

    # create list of poses
    pose_list = []  
    pose_list.append(pose1)
    pose_list.append(pose2)

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
            print(f"\nT matrix: \n{robot.get_current_fk()}")   # print current forward kinematics
            print(f"\nEE task space pose: \n{robot.get_ee_pos(robot.get_joints_readings()[0])}") # print ee xyzpy based on current joint angles
            time.sleep(0.5)
    
    print("\nMoved to target pose.")    # Finished movement messages

    # Shutdown (ALWAYS LAST)
    robot.close()

if __name__ == "__main__":
    main()
