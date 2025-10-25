# EE471 Lab 2_2 10/7/2025
# William Taing wtaing@calpoly.edu
# Description: for lab2 section 2 testing(Using the get_fk() method, write a Python script named lab2_2.py to calculate the forward kinematics (i.e. a numeric 4 × 4 matrix T base ee ) for input joint variable vectors)

from classes.Robot import Robot
import numpy as np

def main():
    home = [0, 0, 0, 0]
    pose1 = [15, -45, -60, 90]
    pose2 = [-90, 15, 30, -45]

    robot = Robot()

    np.set_printoptions(precision=3, suppress=1)

    print(robot.get_fk(home))
    print(robot.get_fk(pose1))
    print(robot.get_fk(pose2))
    print(robot.get_ee_pos(home))
    print(robot.get_ee_pos(pose1))
    print(robot.get_ee_pos(pose2))

if __name__ == "__main__":
    main()