# EE471 Lab 3_3 10/14/2025
# William Taing wtaing@calpoly.edu
# Description: for lab3 section3 testing: Interface and validate your IK solution with the robot

import pickle
import numpy as np
import matplotlib.pyplot as plt

def plot_angle_list(t_list,a_list,title):
        # Convert list to np array
        t_array = np.array(t_list)
        angles_array = np.array(a_list)

        fig, axs = plt.subplots(2,2)    # create subplots
        # Plot motor 1
        axs[0,0].plot(t_array, angles_array[:,0])
        axs[0,0].set_title("Motor 1")
        axs[0,0].set_xlim(0,20)
        # Plot motor 2
        axs[0,1].plot(t_array, angles_array[:,1])
        axs[0,1].set_title("Motor 2")
        axs[0,1].set_xlim(0,20)
        # Plot motor 3
        axs[1,0].plot(t_array, angles_array[:,2])
        axs[1,0].set_title("Motor 3")
        axs[1,0].set_xlim(0,20)
        # Plot motor 4
        axs[1,1].plot(t_array, angles_array[:,3])
        axs[1,1].set_title("Motor 4")
        axs[1,1].set_xlim(0,20)
        # Open plot
        fig.suptitle(title)
        for axs in axs.flat:
            axs.set(xlabel='Time (s)', ylabel='Angle (deg)')
        plt.tight_layout()
        plt.show()

def plot_ee_pose_list(t_list, ee_pose_list, title):
        # Convert lists to arrays
        t_list = np.array(t_list)
        ee_pose_list = np.array(ee_pose_list)

        # Slice x, y, z, alpha
        x = ee_pose_list[:,0]
        y = ee_pose_list[:,1]
        z = ee_pose_list[:,2]
        alpha = ee_pose_list[:,4]
        
        # End-effector pose vs time 
        fig, axs = plt.subplots(2,2)    # create subplots
        # Plot x
        axs[0,0].plot(t_list, x, linestyle='-')
        axs[0,0].set_title("X position vs time")
        axs[0,0].set_ylim(-350,350)
        axs[0,0].set_xlabel("Time (s)")
        axs[0,0].set_ylabel("X (mm)")
        # Plot y
        axs[0,1].plot(t_list, y, linestyle='--')
        axs[0,1].set_title("Y position vs time")
        axs[0,1].set_ylim(-350,350)
        axs[0,1].set_xlabel("Time (s)")
        axs[0,1].set_ylabel("Y (mm)")
        # Plot z
        axs[1,0].plot(t_list, z, linestyle='-.')
        axs[1,0].set_title("Z position vs time")
        axs[1,0].set_ylim(-350,350)
        axs[1,0].set_xlabel("Time (s)")
        axs[1,0].set_ylabel("Z (mm)")
        # Plot alpha
        axs[1,1].plot(t_list, alpha, linestyle=':')
        axs[1,1].set_title("Pitch vs time")
        axs[1,1].set_ylim(-100,100)
        axs[1,1].set_xlabel("Time (s)")
        axs[1,1].set_ylabel("Pitch (deg)")
        # Open plot
        fig.suptitle("End-effector pose vs time")
        plt.tight_layout()
        plt.show()

def plot_3D_trajectory_list(ee_pose_list, title):
        # Convert lists to arrays
        ee_pose_list = np.array(ee_pose_list)

        # Slice x, y, z, alpha
        x = ee_pose_list[:,0]
        y = ee_pose_list[:,1]
        z = ee_pose_list[:,2]
        alpha = ee_pose_list[:,4]

        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot3D(x, y, z)
        ax.set_title(title)
        ax.set_xlabel('x (mm)')
        ax.set_ylabel('y (mm)')
        ax.set_zlabel('z (mm)')
        plt.show()

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

    #robot = Robot()

    np.set_printoptions(precision=3, suppress=1)    # set print precision and suppression

    # Enable torque and set time-based profile
    #robot.write_motor_state(True)
    #robot.write_time(traj_time)
    
    # # wp1
    # print("Homing to wp1 ...")
    # robot.write_joints(robot.get_ik(wp1))
    # time.sleep(traj_time)

    # # Move to target poses
    # t_init = time.perf_counter()
    # for wp in pose_list:  # for all poses in pose_list
    #     print(f"\nMoving to {wp} (mm)(deg) over {traj_time:.1f}s ...")
    #     robot.write_joints(robot.get_ik(wp))    # move robot to pose
    #     t0 = time.perf_counter()
    #     while time.perf_counter() - t0 < traj_time: # while moving to pose
    #         time_list.append(time.perf_counter()-t_init)            # save current time to list
    #         angles_list.append(robot.get_joints_readings()[0])  # save current joint angle to list
    #         ee_list.append(robot.get_ee_pos(robot.get_joints_readings()[0]))    # save current ee pose to list
    #         time.sleep(poll_dt)
    
    
    # print("\nMoved to target pose.")    # Finished movement messages

    # # Save data to pickle file
    # save_to_pickle({"time": time_list, "angles": angles_list, "target_angles": pose_list, "traj_time":traj_time, "ee_pose": ee_list}, "lab3_3_data.pkl")

    # Load data from pickle file
    data = load_from_pickle("lab3_3_data.pkl")
    time_pkl = data["time"]
    angles_pkl = data["angles"]
    ee_pose_pkl = data["ee_pose"]
    
    # End-effector pose vs time
    plot_ee_pose_list(time_pkl, ee_pose_pkl,"End-effector pose vs time")

    # 3D trajectory
    plot_3D_trajectory_list(ee_pose_pkl, "3D Trajectory")

    # Joint angles
    plot_angle_list(time_pkl, angles_pkl,"Motor Angle vs Time")

    # Shutdown (ALWAYS LAST)
    #robot.close()

if __name__ == "__main__":
    main()
