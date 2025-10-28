# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# Skeleton Robot class for OpenManipulator-X Robot for EE 471

# Duplicate of Robot class to run methods without access to physical robot

import numpy as np
import matplotlib.pyplot as plt

"""
Robot class for controlling the OpenManipulator-X Robot.
Inherits from OM_X_arm and provides methods specific to the robot's operation.
"""
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

    """
    Additional Funcitons: (William Taing EE471 Fall2025)

    plot_angle_list(t_list,a_list,title):
        input: time list, angle list, title
        function: 2x2 plot of joint angles vs time 
    plot_samplet_hist(t_list,title):
        input: time list, title
        function: 100 bin histogram of sample intervals
    plot_ee_pose_list(t_list, ee_pose_list, title):
        input: eepose list, title
        funciton: 2x2 plot, 1-3  xyz vs time plots, 4 angle vs time plot
    plot_3D_trajectory_list(ee_pose_list, title):
        input: time list, eepose list, title
        function: plot 3D plot of xyz
    def plot_xyz_posvelacc_list(t_list, ee_pose_list, title):
        inputL time list, eepose, list, title
        function: plot position, velocity, and acceleration over time for x, y, z
    def plot_alpha_posvelacc_list(t_list, ee_pose_list, title):
        inputL time list, eepose, list, title
        function: plot position, velocity, and acceleration over time for alpha
    
    get_dh_row_mat(row):
        input: row[theta, d, a, alpha]
        function: returns np 4x4 array DH homogeneous transform from DH parameters
    get_int_mat(joint_angles):
        input: joint_angles[] (deg)
        function: updates home pose DH table with new joint angles, returns nparray A_stack list of all nparray Ai
    get_fk(joint_angles):
        input: joint_angles[] (deg)
        function: returns Homogeneous transform 4x4 nparray T^0_4 (base to end-effector) given joint angles
    get_current_fk():
        input: none
        function: rturns current T, gets current joint readings and calls get_fk
    get_ee_pos(joint_angles):
        input: joint_angles[] (deg)
        function: returns 5x1 nparray of end effector position & orientation [x,y,z,pitch,yaw] in frame 0
    get_ik(eepose):
        input: eepose[] (mm, deg)
        function: returns 4x1 nparray of joint angles(eblow up) calculated from inverse kinematics given eepose

    """

    def plot_angle_list(t_list,a_list,title):
        # Convert list to np array
        t_array = np.array(t_list)
        angles_array = np.array(a_list)

        fig, axs = plt.subplots(2,2)    # create subplots
        # Plot motor 1
        axs[0,0].plot(t_array, angles_array[:,0])
        axs[0,0].set_title("Motor 1")
        #axs[0,0].set_xlim(0,25)
        # Plot motor 2
        axs[0,1].plot(t_array, angles_array[:,1])
        axs[0,1].set_title("Motor 2")
        #axs[0,1].set_xlim(0,25)
        # Plot motor 3
        axs[1,0].plot(t_array, angles_array[:,2])
        axs[1,0].set_title("Motor 3")
        #axs[1,0].set_xlim(0,25)
        # Plot motor 4
        axs[1,1].plot(t_array, angles_array[:,3])
        axs[1,1].set_title("Motor 4")
        #axs[1,1].set_xlim(0,25)
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
    
    def plot_ee_pose_list(self, t_list, ee_pose_list, title):
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
        fig.suptitle(title)
        plt.tight_layout()
        plt.show()

    def plot_3D_trajectory_list(self, ee_pose_list, title):
        # Convert lists to arrays
        ee_pose_list = np.array(ee_pose_list)

        # Slice x, y, z, alpha
        x = ee_pose_list[:,0]
        y = ee_pose_list[:,1]
        z = ee_pose_list[:,2]
        # alpha = ee_pose_list[:,4]

        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot3D(x, y, z)
        ax.set_title(title)
        ax.set_xlabel('x (mm)')
        ax.set_ylabel('y (mm)')
        ax.set_zlabel('z (mm)')
        plt.show()
    
    def plot_xyz_posvelacc_list(self, t_list, ee_pose_list, title):
        # Convert lists to arrays
        t_list = np.array(t_list)
        ee_pose_list = np.array(ee_pose_list)

        # Slice x, y, z
        x = ee_pose_list[:,0]
        y = ee_pose_list[:,1]
        z = ee_pose_list[:,2]

        # Calculate Velocity and Acceleration
        vx = np.gradient(x)
        vy = np.gradient(y)
        vz = np.gradient(z)
        ax = np.gradient(vx)
        ay = np.gradient(vy)
        az = np.gradient(vz)
        
        # End-effector pose vs time 
        fig, axs = plt.subplots(3,1)    # create subplots
        # Plot position
        axs[0].plot(t_list, x, label='x position')
        axs[0].plot(t_list, y, label='y position')
        axs[0].plot(t_list, z, label='z position')
        axs[0].set_title("XYZ position vs time")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Position (mm)")
        axs[0].legend(loc='upper right')
        # Plot velocity
        axs[1].plot(t_list, vx, label='x velocity')
        axs[1].plot(t_list, vy, label='y velocity')
        axs[1].plot(t_list, vz, label='z velocity')
        axs[1].set_title("XYZ velocity vs time")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Velocity (mm/s)")
        axs[1].legend(loc='upper right')
        # Plot acceleration
        axs[2].plot(t_list, ax, label='x acceleration')
        axs[2].plot(t_list, ay, label='y acceleration')
        axs[2].plot(t_list, az, label='z acceleration')
        axs[2].set_title("XYZ accerlcation vs time")
        axs[2].set_xlabel("Time (s)")
        axs[2].set_ylabel("Acceleration (mm/s\u00B2)")
        axs[2].legend(loc='upper right')
        
        # Open plot
        fig.suptitle(title)
        plt.tight_layout()
        plt.show()
    
    def plot_alpha_posvelacc_list(self, t_list, ee_pose_list, title):
        # Convert lists to arrays
        t_list = np.array(t_list)
        ee_pose_list = np.array(ee_pose_list)

        # Slice alpha
        alpha = ee_pose_list[:,3]

        # Calculate Velocity and Acceleration
        walpha = np.gradient(alpha)
        aalpha = np.gradient(walpha)
        
        # End-effector pose vs time 
        fig, axs = plt.subplots(3,1)    # create subplots
        # Plot position
        axs[0].plot(t_list, alpha, label='alpha angle')
        axs[0].set_title("Alpha angle vs time")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Angle (deg)")
        axs[0].legend(loc='upper right')
        # Plot velocity
        axs[1].plot(t_list, walpha, label='alpha velocity')
        axs[1].set_title("Alpha velocity vs time")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Velocity (deg/s)")
        axs[1].legend(loc='upper right')
        # Plot acceleration
        axs[2].plot(t_list, aalpha, label='alpha acceleration')
        axs[2].set_title("XYZ accerlcation vs time")
        axs[2].set_xlabel("Time (s)")
        axs[2].set_ylabel("Acceleration (deg/s\u00B2)")
        axs[2].legend(loc='upper right')
        
        # Open plot
        fig.suptitle(title)
        plt.tight_layout()
        plt.show()


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
        # A_stack = self.get_int_mat(joint_angles)
        A_stack = self.get_int_mat(joint_angles)

        """"
        T = np.eye(4, dtype=float)
        for i in range(4):
            T = T @ A_stack[:, :, i]
        """
        # Create T matrix
        T = A_stack[0] @ A_stack[1] @ A_stack[2] @ A_stack[3] 
        return T

    def get_current_fk(self):
        return self.get_fk(self.get_joints_readings()[0])   # Returns T matrix from current joint angles from get_joint_readings   

    def get_ee_pos(self, joint_angles):
        xyz = self.get_fk(joint_angles) # get xyz position from T
        py = [-sum(joint_angles[1:4]), joint_angles[0]]  # get p and y from joint angles list
        ee_pos = np.append(xyz[0:3,3], np.array(py, dtype=float))   # create complete xyzpy matrix 
        return ee_pos
    
    def get_ik(self, eepose):
        # Given (x, y, z, alpha), compute two solutions, return elbow up solution (Phi=alpha)
        x = eepose[0]
        y = eepose[1]
        z = eepose[2]
        alpha = np.deg2rad(eepose[3])
        l1 = self.dim[0]
        l2 = self.dim[1]
        l3 = self.dim[2]
        l4 = self.dim[3]

        l21 = 128
        l22 = 24

        # Define variables with multiple solutions
        theta2 = np.zeros(2, dtype=float); theta3 = np.zeros(2, dtype=float); theta4 = np.zeros(2, dtype=float)
        beta = np.zeros(2, dtype=float); gamma = np.zeros(2, dtype=float)

        # Calculate necessary equations
        r = np.sqrt((x**2)+(y**2))
        rw = r-l4*np.cos(alpha)
        zw = z-l1-l4*np.sin(alpha)
        dw = np.sqrt((rw**2)+(zw**2))
        tmu = zw/rw
        cbeta = ((l2**2)+(l3**2)-(dw**2))/(2*l2*l3)
        cgamma = ((dw**2)+(l2**2)-(l3**2))/(2*dw*l2)
        tdelta = l22/l21

        # Calculate non-motor angles
        mu = np.arctan2(zw, rw)
        delta = np.atan2(l22, l21)
        beta[0] = np.atan2(np.sqrt(1-cbeta**2), cbeta)
        beta[1] = np.atan2(-np.sqrt(1-cbeta**2), cbeta)
        gamma[0] = np.atan2(np.sqrt(1-cgamma**2), cgamma)
        gamma[1] = np.atan2(-np.sqrt(1-cgamma**2), cgamma)

        # Calculate Theta values
        theta1 = np.atan2(y, x)
        theta2[0] = (np.pi/2)-delta-gamma[0]-mu
        theta2[1] = (np.pi/2)-delta-gamma[1]-mu
        theta3[0] = (np.pi/2)+delta-beta[0]
        theta3[1] = (np.pi/2)+delta-beta[1]
        theta4[0] = -alpha-theta2[0]-theta3[0]
        theta4[1] = -alpha-theta2[1]-theta3[1]

        # Create joint angles list
        joint_angles = []
        joint_angles.append([theta1, theta2[0], theta3[0], theta4[0]])
        joint_angles.append([theta1, theta2[1], theta3[1], theta4[1]])
        
        # Determine if joint angles are valid
        if all(np.less_equal(np.rad2deg(joint_angles[0]),self.joint_limits_max)) & all(np.greater_equal(np.rad2deg(joint_angles[0]),self.joint_limits_min)):
            return np.rad2deg((np.array(joint_angles[0])))      
        elif all(np.less_equal(np.rad2deg(joint_angles[1]),self.joint_limits_max)) & all(np.greater_equal(np.rad2deg(joint_angles[1]),self.joint_limits_min)):
            return np.rad2deg((np.array(joint_angles[1])))      
        else:
            raise ValueError("Invalid Pose: Joint Angles Outside Physical Limits")




