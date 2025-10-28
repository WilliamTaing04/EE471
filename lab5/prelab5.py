# EE471 PreLab 5 10/26/2025
# William Taing wtaing@calpoly.edu
# Description: for prelab 5 Velocity Kinematics of Robot Manipulators

import numpy as np
import matplotlib.pyplot as plt

class Robot():
    """
    Initialize the Robot class.
    Creates constants and connects via serial. Sets default mode and state.
    """
    def __init__(self):
        super().__init__()

        # Robot Dimensions (in mm)
        self.mDim = [77, 130, 124, 126]
        self.mOtherDim = [128, 24]

        """
        Initialize robot constants and the DH table.
        """
        self.dim = [77, 130, 124, 126]  # Physical robot arm lengths in mm

        # DH table all theta=0 (home position)
        self.DH_table = [
                        [0, self.dim[0], 0, (-0.5*np.pi)],
                        [-((0.5*np.pi)-np.arcsin(24/130)), 0, self.dim[1], 0],
                        [(0.5*np.pi)-np.arcsin(24/130), 0, self.dim[2], 0],
                        [0, 0, self.dim[3], 0]
                        ]
        
        self.joint_limits_max = [90, 90, 75, 100]   # Joint max angles (deg)
        self.joint_limits_min = [-90, -120, -90, -100]   # Joint min angles (deg)
    
    def get_dh_row_mat(self, row):
        """
        Compute the Standard DH homogeneous transform A_i from a single DH row.

        Parameters
        ----------
        row : array-like, shape (4,)
            [theta, d, a, alpha] for one joint.

        Returns
        -------
        A : nparray, shape (4,4)
        """

        # TODO: implement A
        # A = np.array([...], dtype=float)
        # return A
        row = np.array(row, dtype=float)
        A = np.array([
                        [np.cos(row[0]), -np.sin(row[0])*np.cos(row[3]), np.sin(row[0])*np.sin(row[3]), row[2]*np.cos(row[0])],
                        [np.sin(row[0]), np.cos(row[0])*np.cos(row[3]), -np.cos(row[0])*np.sin(row[3]), row[2]*np.sin(row[0])],
                        [0, np.sin(row[3]), np.cos(row[3]), row[1]],
                        [0,0,0,1]
                        ]
                        , dtype=float)
        return A

    def get_int_mat(self, joint_angles):
        """
        Build all intermediate DH transforms A_i for the provided joint angles.
        Parameters
        ----------
        joint_angles : array-like, shape (4,)
            Joint variables q in degrees: [q1, q2, q3, q4].

        Returns
        -------
        A_stack : ndarray, shape (4,4,4)
            A_stack[:, :, i] = A_{i+1}

        Steps
        -----
        1) Copy the base DH table.
        2) Add q (deg) to the theta column (col 0).
        3) For each row, compute A_i via get_dh_row_mat(...).
        """

        # TODO: add q to theta column (degrees)
        # dh[:, 0] += q_deg
        dh = np.array(self.DH_table, dtype=float)
        dh[:, 0] += np.deg2rad(np.array(joint_angles), dtype=float) # add joint angles in rad to dh table

        # TODO: build A_stack
        A_stack = []
        for row in dh:
            A_stack.append(self.get_dh_row_mat(row))

        # return A_stack
        return np.array(A_stack, dtype=float)

    def get_jacobian(self, joint_angles):
        # Calculate Ai matrices
        A_stack = self.get_int_mat(joint_angles)
        # Calculate Ti matrices
        T1 = A_stack[0]
        T2 = T1 @ A_stack[1]
        T3 = T2 @ A_stack[2]
        T4 = T3 @ A_stack[3]
        # Slice Zi and Oi
        Z0 = np.transpose(np.array([0, 0, 1]))
        Z1 = T1[0:3, 2]
        Z2 = T2[0:3, 2]
        Z3 = T3[0:3, 2]
        O0 = np.transpose(np.array([0, 0, 0]))
        O1 = T1[0:3, 3]
        O2 = T2[0:3, 3]
        O3 = T3[0:3, 3]
        O4 = T4[0:3, 3]

        # Calculate J
        J = np.zeros((6,4), dtype=float)
        J1 = np.zeros((6,1), dtype=float)
        J2 = np.zeros((6,1), dtype=float)
        J3 = np.zeros((6,1), dtype=float)
        J4 = np.zeros((6,1), dtype=float)


        J1[0:3, 0] = np.transpose(np.cross(Z0, O4 - O0))
        J1[3:6, 0] = Z0
        J2[0:3, 0] = np.cross(Z1, O4 - O1)
        J2[3:6, 0] = Z1
        J3[0:3, 0] = np.cross(Z2, O4 - O2)
        J3[3:6, 0] = Z2
        J4[0:3, 0] = np.cross(Z3, O4 - O3)
        J4[3:6, 0] = Z3

        J = np.hstack((J1, J2, J3, J4))
        return J



def validation():
    robot = Robot()
    q = [0, -10.62, -79.38, 0]
    home = [0, 0, 0, 0]
    
    np.set_printoptions(precision=3, suppress=1)    # set print precision and suppression

    print("Overhead singularity configuration Jacobian")
    print(robot.get_jacobian(q))
    print("Home configuration Jacobian")
    print(robot.get_jacobian(home))

    
if __name__ == "__main__":
    validation()
