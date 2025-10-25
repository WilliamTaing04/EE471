import numpy as np

class Robot:
    def __init__(self):
        """
        Initialize robot constants and the DH table.
        """
        # TODO: set any link lengths you need (in mm), if helpful
        self.dim = [77, 130, 124, 126]

        # TODO: fill your DH table (4x4) based on your derivation.
        self.DH_table = [
                        [0, self.dim[0], 0, (-0.5*np.pi)],
                        [-((0.5*np.pi)-np.arcsin(24/130)), 0, self.dim[1], 0],
                        [(0.5*np.pi)-np.arcsin(24/130), 0, self.dim[2], 0],
                        [0, 0, self.dim[3], 0]
                        ]

    def get_dh_row_mat(self, row):
        """
        Compute the Standard DH homogeneous transform A_i from a single DH row.

        Parameters
        ----------
        row : array-like, shape (4,)
            [theta, d, a, alpha] for one joint.

        Returns
        -------
        A : ndarray, shape (4,4)
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
    
    def get_fk(self, joint_angles):
        """
        Forward kinematics to the end-effector.

        Parameters
        ----------
        joint_angles : array-like, shape (4,)
            Joint variables q in degrees.

        Returns
        -------
        T : ndarray, shape (4,4)
            Homogeneous transform T^0_4 (base to end-effector).
        """
        # TODO:
        # A_stack = self.get_int_mat(joint_angles)
        A_stack = self.get_int_mat(joint_angles)

        """"
        T = np.eye(4, dtype=float)
        for i in range(4):
            T = T @ A_stack[:, :, i]
        """

        # Create T matrix
        T = A_stack[0] @ A_stack[1] @ A_stack[2] @ A_stack[3] 
        # return T
        return T

    def get_current_fk(self):
        return self.get_fk(self.get_joints_readings()[0])   # Returns T matrix from current joint angles    

    def get_ee_pos(self, joint_angles):
        xyz = self.get_fk(joint_angles) # get xyz position from T
        py = [sum(joint_angles[1:4]), joint_angles[0]]  # get p and y from joint angles list
        ee_pos = np.append(xyz[0:3,3], np.array(py, dtype=float))
        return ee_pos