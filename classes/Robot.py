# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# Skeleton Robot class for OpenManipulator-X Robot for EE 471

import numpy as np
from .OM_X_arm import OM_X_arm
from .DX_XM430_W350 import DX_XM430_W350
import matplotlib.pyplot as plt

"""
Robot class for controlling the OpenManipulator-X Robot.
Inherits from OM_X_arm and provides methods specific to the robot's operation.
"""
class Robot(OM_X_arm):
    """
    Initialize the Robot class.
    Creates constants and connects via serial. Sets default mode and state.
    """
    def __init__(self):
        super().__init__()

        self.GRIP_OPEN_DEG  = -45.0
        self.GRIP_CLOSE_DEG = +45.0
        self.GRIP_THRESH_DEG = 180.0

        # Robot Dimensions (in mm)
        self.mDim = [77, 130, 124, 126]
        self.mOtherDim = [128, 24]
        
        # Set default mode and state
        # Change robot to position mode with torque enabled by default
        # Feel free to change this as desired
        self.write_mode('position')
        self.write_motor_state(True)

        # Set the robot to move between positions with a 5 second trajectory profile
        # change here or call writeTime in scripts to change
        self.write_time(5)

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

    def _set_time_profile_bit_all(self, enable: bool):
        """Turn the Drive Mode 'time-based profile' bit (bit 2) on/off for all joints."""
        DX = DX_XM430_W350
        # Read current drive modes
        dm = self.bulk_read_write(DX.DRIVE_MODE_LEN, DX.DRIVE_MODE, None)  # list[int]
        if not isinstance(dm, list) or len(dm) != len(self.motorIDs):
            raise RuntimeError("Failed to read DRIVE_MODE for all joints.")
        new_dm = []
        for v in dm:
            if enable:
                new_dm.append(v | 0b100)   # set bit 2
            else:
                new_dm.append(v & ~0b100)  # clear bit 2
        # Write back (bulk)
        self.bulk_read_write(DX.DRIVE_MODE_LEN, DX.DRIVE_MODE, new_dm)

    """
    Sends the joints to the desired angles.
    Parameters:
    goals (list of 1x4 float): Angles (degrees) for each of the joints to go to.
    """
    def write_joints(self, q_deg):
        """Send joint target angles in degrees (list/array length N)."""
        DX = DX_XM430_W350
        q_deg = list(q_deg)
        if len(q_deg) != len(self.motorIDs):
            raise ValueError(f"Expected {len(self.motorIDs)} joint angles, got {len(q_deg)}")

        ticks = [int(round(angle * DX.TICKS_PER_DEG + DX.TICK_POS_OFFSET)) for angle in q_deg]

        # If you're in normal position mode (not extended), keep values in [0, 4095]
        ticks = [max(0, min(int(DX.TICKS_PER_ROT - 1), t)) for t in ticks]

        self.bulk_read_write(DX.POS_LEN, DX.GOAL_POSITION, ticks)

    """
    Creates a time-based profile (trapezoidal) based on the desired times.
    This will cause write_position to take the desired number of seconds to reach the setpoint.
    Parameters:
    time (float): Total profile time in seconds. If 0, the profile will be disabled (be extra careful).
    acc_time (float, optional): Total acceleration time for ramp up and ramp down (individually, not combined). Defaults to time/3.
    """
    def write_time(self, total_time_s, acc_time_s=None):
        """Configure trapezoidal TIME profile for all joints."""
        if acc_time_s is None:
            acc_time_s = float(total_time_s) / 3.0

        # Enable time-based profile (bit 2) for all joints
        self._set_time_profile_bit_all(True)

        acc_ms = int(round(acc_time_s * DX_XM430_W350.MS_PER_S))
        tot_ms = int(round(float(total_time_s) * DX_XM430_W350.MS_PER_S))

        # Bulk write to all joints
        self.bulk_read_write(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, [acc_ms] * len(self.motorIDs))
        self.bulk_read_write(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, [tot_ms] * len(self.motorIDs))

    """
    Sets the gripper to be open or closed.
    Parameters:
    open (bool): True to set the gripper to open, False to close.
    """
    def write_gripper(self, is_open: bool):
        """Open/close gripper using fixed angles in position mode."""
        target = self.GRIP_OPEN_DEG if is_open else self.GRIP_CLOSE_DEG
        self.gripper.write_position(target)

    def read_gripper(self) -> float:
        """Return gripper joint position in degrees."""
        return self.gripper.read_position()

    def read_gripper_open(self) -> bool:
        return (self.read_gripper() > self.GRIP_THRESH_DEG)

    """
    Sets position holding for the joints on or off.
    Parameters:
    enable (bool): True to enable torque to hold the last set position for all joints, False to disable.
    """
    def write_motor_state(self, enable):
        state = 1 if enable else 0
        states = [state] * self.motorsNum  # Repeat the state for each motor
        self.bulk_read_write(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, states)

    """
    Supplies the joints with the desired currents.
    Parameters:
    currents (list of 1x4 float): Currents (mA) for each of the joints to be supplied.
    """
    def write_currents(self, currents):
        current_in_ticks = [round(current * DX_XM430_W350.TICKS_PER_mA) for current in currents]
        self.bulk_read_write(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, current_in_ticks)

    """
    Change the operating mode for all joints.
    Parameters:
    mode (str): New operating mode for all joints. Options include:
        "current": Current Control Mode (writeCurrent)
        "velocity": Velocity Control Mode (writeVelocity)
        "position": Position Control Mode (writePosition)
        "ext position": Extended Position Control Mode
        "curr position": Current-based Position Control Mode
        "pwm voltage": PWM Control Mode
    """
    def write_mode(self, mode):
        if mode in ['current', 'c']:
            write_mode = DX_XM430_W350.CURR_CNTR_MD
        elif mode in ['velocity', 'v']:
            write_mode = DX_XM430_W350.VEL_CNTR_MD
        elif mode in ['position', 'p']:
            write_mode = DX_XM430_W350.POS_CNTR_MD
        elif mode in ['ext position', 'ep']:
            write_mode = DX_XM430_W350.EXT_POS_CNTR_MD
        elif mode in ['curr position', 'cp']:
            write_mode = DX_XM430_W350.CURR_POS_CNTR_MD
        elif mode in ['pwm voltage', 'pwm']:
            write_mode = DX_XM430_W350.PWM_CNTR_MD
        else:
            raise ValueError(f"writeMode input cannot be '{mode}'. See implementation in DX_XM430_W350 class.")

        self.write_motor_state(False)
        write_modes = [write_mode] * self.motorsNum  # Create a list with the mode value for each motor
        self.bulk_read_write(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, write_modes)
        self.write_motor_state(True)

    """
    Gets the current joint positions, velocities, and currents.
    Returns:
    numpy.ndarray: A 3x4 array containing the joints' positions (deg), velocities (deg/s), and currents (mA).
    """
    def get_joints_readings(self):
        """
        Returns a 3xN array: [deg; deg/s; mA] for the N arm joints (excludes gripper).
        """
        N = len(self.motorIDs)

        # Bulk read raw registers
        pos_u32 = self.bulk_read_write(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION, None)  # list of ints
        vel_u32 = self.bulk_read_write(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY, None)
        cur_u16 = self.bulk_read_write(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT,  None)

        # Vectorize
        pos_u32 = np.array(pos_u32, dtype=np.uint32)
        vel_u32 = np.array(vel_u32, dtype=np.uint32)  # signed 32-bit
        cur_u16 = np.array(cur_u16, dtype=np.uint16)  # signed 16-bit

        # Convert signed types
        vel_i32 = (vel_u32.astype(np.int64) + (1 << 31)) % (1 << 32) - (1 << 31)
        vel_i32 = vel_i32.astype(np.int32)
        cur_i16 = (cur_u16.astype(np.int32) + (1 << 15)) % (1 << 16) - (1 << 15)
        cur_i16 = cur_i16.astype(np.int16)

        # Units
        q_deg  = (pos_u32.astype(np.int64) - int(DX_XM430_W350.TICK_POS_OFFSET)) / DX_XM430_W350.TICKS_PER_DEG
        qd_dps = vel_i32 / DX_XM430_W350.TICKS_PER_ANGVEL
        I_mA   = cur_i16 / DX_XM430_W350.TICKS_PER_mA

        readings = np.vstack([q_deg.astype(float), qd_dps.astype(float), I_mA.astype(float)])
        return readings

    """
    Sends the joints to the desired velocities.
    Parameters:
    vels (list of 1x4 float): Angular velocities (deg/s) for each of the joints to go at.
    """
    def write_velocities(self, vels):
        """Send joint target velocities in deg/s (list/array length N)."""
        vels = list(vels)
        if len(vels) != len(self.motorIDs):
            raise ValueError(f"Expected {len(self.motorIDs)} velocities, got {len(vels)}")

        ticks_per_s = [int(round(v * DX_XM430_W350.TICKS_PER_ANGVEL)) for v in vels]  # signed
        self.bulk_read_write(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, ticks_per_s)

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
    plot_ee_velocity_list(t_list, ee_velocity_list, title):
        input: eepose list, title
        funciton: 2x2 plot, 1-3  xyz vs time plots, 4 angle vs time plot
    plot_angle_velocity_list(t_list, angles_velocity_list, title):
        input: angles velocities list, title
        funciton: 2x2 plot of joint angles velocities vs time 
    plot_ee_pose_desvsacc_list(t_list, ee_pose_list, ee_pose_list_des, title):
        input: time list, actual eepose, desired eepose, title
        function: 3x1 plot of xyz actual overlaid with desired
    plot_ee_pose_error_list(t_list, error_list, sse, title):
        input: time list, error list, title
        functtion: 3x1 plot of xyz error vs time
    plot_control_output_list(t_list, control_output_list, title):
        inputl time list, control output list, title
        function: 3x1 plot of xyz control output of PID mm/s



    
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
    get_jacobian(joint_angles):
        input: joint_angles[] (deg)
        function: returns 6x4 nparray Jacobian Matrix
    get_fwd_vel_kin(joint_angles, joint_velocities)
        input: joint_angles[] (deg), joint_velocities[] (rad/s)
        function: returns 6x1 nparray of ee velocites in (mm/s) (deg/s)
     

    """

    def plot_angle_list(t_list,a_list,title):
        # Convert list to np array
        t_array = np.array(t_list)
        angles_array = np.array(a_list)

        fig, axs = plt.subplots(2,2)    # create subplots
        # Plot joint 1
        axs[0,0].plot(t_array, angles_array[:,0])
        axs[0,0].set_title("Joint 1")
        #axs[0,0].set_xlim(0,25)
        # Plot joint 2
        axs[0,1].plot(t_array, angles_array[:,1])
        axs[0,1].set_title("Joint 2")
        #axs[0,1].set_xlim(0,25)
        # Plot joint 3
        axs[1,0].plot(t_array, angles_array[:,2])
        axs[1,0].set_title("Joint 3")
        #axs[1,0].set_xlim(0,25)
        # Plot joint 4
        axs[1,1].plot(t_array, angles_array[:,3])
        axs[1,1].set_title("Joint 4")
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
        alpha = ee_pose_list[:,3]
        
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

    def plot_ee_velocity_list(t_list, ee_velocity_list, title):
        # Convert lists to arrays
        t_list = np.array(t_list)
        ee_velocity_list = np.array(ee_velocity_list)

        # Slice x, y, z, alpha
        vx = ee_velocity_list[:,0]
        vy = ee_velocity_list[:,1]
        vz = ee_velocity_list[:,2]
        valpha = ee_velocity_list[:,4]
        
        # End-effector velocity vs time 
        fig, axs = plt.subplots(2,2)    # create subplots
        # Plot vx
        axs[0,0].plot(t_list, vx)
        axs[0,0].set_title("X velocity vs time")
        # axs[0,0].set_ylim(-350,350)
        axs[0,0].set_xlabel("Time (s)")
        axs[0,0].set_ylabel("X Velocity (mm/s)")
        # Plot vy
        axs[0,1].plot(t_list, vy)
        axs[0,1].set_title("Y velocity vs time")
        # axs[0,1].set_ylim(-350,350)
        axs[0,1].set_xlabel("Time (s)")
        axs[0,1].set_ylabel("Y Velocity (mm/s)")
        # Plot vz
        axs[1,0].plot(t_list, vz)
        axs[1,0].set_title("Z velocity vs time")
        # axs[1,0].set_ylim(-350,350)
        axs[1,0].set_xlabel("Time (s)")
        axs[1,0].set_ylabel("Z Velocity (mm/s)")
        # Plot valpha
        axs[1,1].plot(t_list, valpha)
        axs[1,1].set_title("Alpha velocity vs time")
        # axs[1,1].set_ylim(-100,100)
        axs[1,1].set_xlabel("Time (s)")
        axs[1,1].set_ylabel("Pitch Velocity (deg/s)")
        # Open plot
        fig.suptitle(title)
        plt.tight_layout()
        plt.show()

    def plot_angle_velocity_list(t_list, angles_velocity_list, title):
        # Convert list to np array
        t_array = np.array(t_list)
        joint_angles_array = np.array(angles_velocity_list)

        fig, axs = plt.subplots(2,2)    # create subplots
        # Plot joint 1
        axs[0,0].plot(t_array, joint_angles_array[:,0])
        axs[0,0].set_title("Joint 1 Velocity")
        #axs[0,0].set_xlim(0,25)
        # Plot joint 2
        axs[0,1].plot(t_array, joint_angles_array[:,1])
        axs[0,1].set_title("Joint 2 Velocity")
        #axs[0,1].set_xlim(0,25)
        # Plot joint 3
        axs[1,0].plot(t_array, joint_angles_array[:,2])
        axs[1,0].set_title("Joint 3 Velocity")
        #axs[1,0].set_xlim(0,25)
        # Plot joint 4
        axs[1,1].plot(t_array, joint_angles_array[:,3])
        axs[1,1].set_title("Joint 4 Velocity")
        #axs[1,1].set_xlim(0,25)
        # Open plot
        fig.suptitle(title)
        for axs in axs.flat:
            axs.set(xlabel='Time (s)', ylabel='Angle Velocity (deg/s)')
        plt.tight_layout()
        plt.show()

    def plot_ee_pose_desvsacc_list(self, t_list, ee_pose_list, ee_pose_list_des, title):
        # Convert lists to arrays
        t_list = np.array(t_list)
        ee_pose_list = np.array(ee_pose_list)
        ee_pose_list_des = np.array(ee_pose_list_des)

        # Slice x, y, z, alpha
        x = ee_pose_list[:,0]
        y = ee_pose_list[:,1]
        z = ee_pose_list[:,2]
        xdes = ee_pose_list_des[:,0]
        ydes = ee_pose_list_des[:,1]
        zdes = ee_pose_list_des[:,2]
        
        # End-effector pose actual and desired vs time 
        fig, axs = plt.subplots(3,1)    # create subplots
        # Plot x
        axs[0].plot(t_list, x, linestyle='-', label='actual pos')
        axs[0].plot(t_list, xdes, linestyle='--', label='desired pos')
        axs[0].set_title("X position vs time")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("X (mm)")
        axs[0].legend(loc='upper left')
        # Plot y
        axs[1].plot(t_list, y, linestyle='-', label='actual pos')
        axs[1].plot(t_list, ydes, linestyle='--', label='desired pos')
        axs[1].set_title("Y position vs time")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Y (mm)")
        axs[1].legend(loc='upper left')
        # Plot z
        axs[2].plot(t_list, z, linestyle='-', label='actual pos')
        axs[2].plot(t_list, zdes, linestyle='--', label='desired pos')
        axs[2].set_title("Z position vs time")
        axs[2].set_xlabel("Time (s)")
        axs[2].set_ylabel("Z (mm)")
        axs[2].legend(loc='upper left')
        # Open plot
        fig.suptitle(title)
        plt.tight_layout()
        plt.show()

    def plot_ee_pose_error_list(self, t_list, error_list, sse, title):
        # Convert lists to arrays
        t_list = np.array(t_list)
        error_list = np.array(error_list)

        # Slice x, y, z
        ex = error_list[:,0]
        ey = error_list[:,1]
        ez = error_list[:,2]
        
        # End-effector pose eror vs time 
        fig, axs = plt.subplots(3,1)    # create subplots
        # Plot x
        axs[0].plot(t_list, ex)
        axs[0].set_title("X error vs time")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("X (mm)")
        axs[0].axhline(y=sse, color='r', linestyle='--', linewidth=1)
        axs[0].axhline(y=-sse, color='r', linestyle='--', linewidth=1)
        # Plot y
        axs[1].plot(t_list, ey)
        axs[1].set_title("Y error vs time")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Y (mm)")
        axs[1].axhline(y=sse, color='r', linestyle='--', linewidth=1)
        axs[1].axhline(y=-sse, color='r', linestyle='--', linewidth=1)
        # Plot z
        axs[2].plot(t_list, ez)
        axs[2].set_title("Z error vs time")
        axs[2].set_xlabel("Time (s)")
        axs[2].set_ylabel("Z (mm)")
        axs[2].axhline(y=sse, color='r', linestyle='--', linewidth=1)
        axs[2].axhline(y=-sse, color='r', linestyle='--', linewidth=1)
        # Open plot
        fig.suptitle(title)
        plt.tight_layout()
        plt.show()
    
    def plot_control_output_list(self, t_list, control_output_list, title):
        # Convert lists to arrays
        t_list = np.array(t_list)
        control_output_list = np.array(control_output_list)

        # Slice x, y, z
        cx = control_output_list[:,0]
        cy = control_output_list[:,1]
        cz = control_output_list[:,2]
        
        # End-effector pose eror vs time 
        fig, axs = plt.subplots(3,1)    # create subplots
        # Plot x
        axs[0].plot(t_list, cx)
        axs[0].set_title("X control vs time")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("X vel(mm/s)")
        # Plot y
        axs[1].plot(t_list, cy)
        axs[1].set_title("Y control vs time")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Y vel(mm/s)")
        # Plot z
        axs[2].plot(t_list, cz)
        axs[2].set_title("Z control vs time")
        axs[2].set_xlabel("Time (s)")
        axs[2].set_ylabel("Z vel(mm/s)")
        # Open plot
        fig.suptitle(title)
        plt.tight_layout()
        plt.show()


    #----------------------------------------------------------------------------------------------#
    #----------------------------------------------------------------------------------------------#
    #----------------------------------------------------------------------------------------------#


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

        # Calculate non-joint angles
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
    
    def get_fwd_vel_kin(self, joint_angles, joint_velocities):
        # Get Jacobian
        J = self.get_jacobian(joint_angles)
        # Calculate EE velocity 
        P = J @ np.array(joint_velocities)
        return P






