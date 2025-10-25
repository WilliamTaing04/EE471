# EE471 PreLab 4 10/19/2025
# William Taing wtaing@calpoly.edu
# Description: for prelab 4 Cubic Trajectory Generation for the OpenManipulator-X Robot Arm verification

# # Usage Example
# import numpy as np
# from traj_planner import TrajPlanner

# # Define setpoints (example values)
# setpoints = np.array([
#     [15, -45, -60, 90],
#     [-90, 15, 30, -45]
# ])

# # Create a TrajPlanner object
# trajectories = TrajPlanner(setpoints)

# # Generate cubic trajectory
# cubic_traj = trajectories.get_cubic_traj(traj_time=5, points_num=10)
# print(cubic_traj)

from TrajPlanner import TrajPlanner
import numpy as np

def main():
    np.set_printoptions(precision=3, suppress=1)    # set print precision and suppression

    # Create setpoints
    setpoints = np.array([[15, -45, -60, 90],   # (deg)
                          [-90, 15, 30, -45]
                          ])
    traj_time = 5   # (sec)
    points_num = 6

    tp = TrajPlanner(setpoints) # Create TrajPlanner object
    print(tp.get_cubic_traj(traj_time, points_num)) # Print get_cubic_traj

    


if __name__ == "__main__":
    main()