# EE471 PreLab 4 10/17/2025
# William Taing wtaing@calpoly.edu
# Description: for prelab 4 Cubic Trajectory Generation for the OpenManipulator-X Robot Arm

import numpy as np

class TrajPlanner:
    """
    Trajectory Planner class for calculating trajectories for different polynomial orders and relevant coefficients.
    """

    def __init__(self, setpoints):
        """
        Initialize the TrajPlanner class.

        Parameters:
        setpoints (numpy array): List of setpoints to travel to.
        """
        self.setpoints = setpoints

    ## Implement the required methods below. ##
    def calc_cubic_coeff(self, t0, tf, p0, pf, v0, vf):
        """
        Given the initial time, final time, initial position, final position, initial velocity, and final velocity,
        returns cubic polynomial coefficients.

        Parameters:
        t0 (float): Start time of trajectory
        tf (float): End time of trajectory
        p0 (float): Initial setpoint
        pf (float): Final setpoint
        v0 (float): Initial velocity
        vf (float): Final velocity

        Returns:
        numpy array: The calculated polynomial coefficients.
        """
        
        # Create matrices for known constraints
        A = np.array(
        [[1, t0, t0**2, t0**3],
                [0, 1, 2*t0, 3*t0**2],
                [1, tf, tf**2, tf**3],
                [0, 1, 2*tf, 3*tf**2]])
        b = np.array([p0, v0, pf, vf])

        # Find unique solution
        a = np.linalg.solve(A, b)   # a = np.linalg.inv(A) @ b
        print(np.array(a))      # Print a coeff
        return np.array(a)

    def calc_cubic_traj(self, traj_time, points_num, coeff):
        """
        Given the time between setpoints, number of points between waypoints, and polynomial coefficients,
        returns the cubic trajectory of waypoints for a single pair of setpoints.

        Parameters:
        traj_time (int): Time between setPoints.
        points_num (int): Number of waypoints between setpoints.
        coeff (numpy array): Polynomial coefficients for trajectory.

        Returns:
        numpy array: The calculated waypoints.
        """

        # Create array of times from 0 to traj_time with points_num number of points +2
        t_array = np.linspace(0, traj_time, points_num+2)

        # Create waypoints array wp = a0 + a1*t + a2*t^2 + a3^t^3
        waypoints = coeff[0] + coeff[1]*t_array + coeff[2]*t_array**2 + coeff[3]*t_array**3

        return np.array(waypoints)


    def get_cubic_traj(self, traj_time, points_num):
        """
        Given the time between setpoints and number of points between waypoints, returns the cubic trajectory.

        Parameters:
        traj_time (int): Time between setPoints.
        points_num (int): Number of waypoints between setpoints.

        Returns:
        numpy array: List of waypoints for the cubic trajectory.
        """
        
        # Create waypoints list
        waypoints = []

        # Create array of times from 0 to traj_time with points_num number of points +2
        t_array = np.linspace(0, traj_time, points_num+2)
        
        for joint in range(len(self.setpoints[0])):        # for each joint
                coeff = self.calc_cubic_coeff(0, traj_time, self.setpoints[0][joint], self.setpoints[1][joint], 0, 0)     # find coeffs
                waypoints.append(self.calc_cubic_traj(traj_time, points_num, coeff))      # find waypoints
        waypoints.append(t_array)       # Include array of times

        return np.array(waypoints)
                    




            
            

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
