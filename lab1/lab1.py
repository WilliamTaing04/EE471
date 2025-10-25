# EE471 Lab 1 9/30/2025
# William Taing wtaing@calpoly.edu
# Description: Multiple Joint manipulation, determining minimum sample time, numpy arrays and matlab plots, pickle file

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

def plot_angle_list(t_list,a_list,title):
    # Convert list to np array
    t_array = np.array(t_list)
    angles_array = np.array(a_list)

    fig, axs = plt.subplots(2,2)    # create subplots
    # Plot motor 1
    axs[0,0].plot(t_array, angles_array[:,0])
    axs[0,0].set_title("Motor 1")
    axs[0,0].set_xlim(0,10)
    # Plot motor 2
    axs[0,1].plot(t_array, angles_array[:,1])
    axs[0,1].set_title("Motor 2")
    axs[0,1].set_xlim(0,10)
    # Plot motor 3
    axs[1,0].plot(t_array, angles_array[:,2])
    axs[1,0].set_title("Motor 3")
    axs[1,0].set_xlim(0,10)
    # Plot motor 4
    axs[1,1].plot(t_array, angles_array[:,3])
    axs[1,1].set_title("Motor 4")
    axs[1,1].set_xlim(0,10)
    # Open plot
    fig.suptitle(title)
    for axs in axs.flat:
        axs.set(xlabel='Time (s)', ylabel='Angle (deg)')
    plt.tight_layout()
    plt.show()

def plot_samplet_hist(t_list,title):
    # Convert list to np array
    t_array = np.array(t_list)
    
    # Derive sample intervals
    delta_t=np.diff(t_list)

    # Calculate basic statistics
    avg_dt = np.mean(delta_t)
    max_dt = np.max(delta_t)
    min_dt = np.min(delta_t)
    stddev_dt = np.std(delta_t)
    print(f"Average Sample Time: {avg_dt: .4f}s")
    print(f"Max Sample Time: {max_dt: .4f}s")
    print(f"Min Sample Time: {min_dt: .4f}s")
    print(f"Standard Deviation Sample Time: {stddev_dt: .4f}s")

    # Plot histogram
    plt.figure(figsize=(8, 5))
    plt.xlim(0.04, 0.06)
    plt.hist(delta_t, bins=100)
    plt.title(title)
    plt.xlabel("Sample Time (S)")
    plt.ylabel("Frequency")
    plt.show()

def main():
    traj_time = 2.0                  # sec
    target_pose = [45, -30, 30, 75]  # deg
    robot = Robot()

    time_list = []          # time list
    angles_list = []        # motor angles list

    # Enable torque and set time-based profile
    robot.write_motor_state(True)
    robot.write_time(traj_time)

    # Home
    print("Homing to [0, 0, 0, 0] deg ...")
    robot.write_joints([0, 0, 0, 0])
    time.sleep(traj_time)

    # Move to target pose
    print(f"\nMoving base to target pose over {traj_time:.1f}s ...")
    robot.write_joints(target_pose)
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < traj_time:
        time_list.append(time.perf_counter()-t0)            # save current time to list
        angles_list.append(robot.get_joints_readings()[0])  # save current joint angle to list
    
    print("\nMoved to target pose.")    # Finished movement messages

    # Create plots for 10s
    plot_angle_list(time_list,angles_list,"Motor Angle vs Time: 10s Traj Time")
    plot_samplet_hist(time_list,"Distribution of Sample Intervals: 10s Traj Time")

    # Save data to pickle file
    save_to_pickle({"time": time_list, "angles": angles_list, "target_angles": target_pose, "traj_time":traj_time}, "lab1_data_2s.pkl")

    # Load data from pickle file 10s
    data10 = load_from_pickle("lab1_data_10s.pk1")
    t_pickle10 = data10["timestamps_s"]
    q_pickle10 = data10["joint_deg"]
    # Load data from pickle file 10s
    data2 = load_from_pickle("lab1_data_2s.pk1")
    t_pickle2 = data2["timestamps_s"]
    q_pickle2 = data2["joint_deg"]
    # Create plots for 10s from pickle
    plot_angle_list(t_pickle10,q_pickle10,"Motor Angle vs Time: 10s Traj Time")
    plot_samplet_hist(t_pickle10,"Distribution of Sample Intervals: 10s Traj Time")
    # Create plots for 2s from pickle
    plot_angle_list(t_pickle2,q_pickle2,"Motor Angle vs Time: 2s Traj Time")
    plot_samplet_hist(t_pickle2,"Distribution of Sample Intervals: 2s Traj Time")

    # Shutdown (ALWAYS LAST)
    robot.close()

if __name__ == "__main__":
    main()
