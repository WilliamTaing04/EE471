# EE471 Prelab3 10/13/2025
# William Taing wtaing@calpoly.edu
# Description: Create and verify inverse kinematic function
import numpy as np

class Robot:
    def __init__(self):
        """
        Initialize robot constants and the DH table.
        """
        # TODO: set any link lengths you need (in mm), if helpful
        self.dim = [77, 130, 124, 126] 

        self.joint_limits_max = [90, 90, 75, 100]   # Joint max angles (deg)
        self.joint_limits_min = [-90, -120, -90, -100]   # Joint min angles (deg)

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

        # Create
        joint_angles = []
        joint_angles.append([theta1, theta2[0], theta3[0], theta4[0]])
        joint_angles.append([theta1, theta2[1], theta3[1], theta4[1]])
        

        # Determine elbow up configuration via theta3 angle, then return joint angles(deg)
        if theta3[0]<0:
            return np.rad2deg((np.array(joint_angles[0])))
        elif theta3[1]<0:
            return np.rad2deg((np.array(joint_angles[1])))    
        else:
            return np.rad2deg((np.array(joint_angles[0])))


def main():
    case1 = [274, 0 ,204, 0]
    case2 = [16, 4, 336, 15]
    case3 = [0, -270, 106, 0]
    np.set_printoptions(precision=3, suppress=1)

    robot = Robot()

    print(robot.get_ik(case1))
    print(robot.get_ik(case2))
    print(robot.get_ik(case3))


if __name__ == "__main__":
    main()