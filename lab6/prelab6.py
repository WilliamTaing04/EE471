# EE471 Prelab 6 11/3/2025
# William Taing wtaing@calpoly.edu
# Description: for Prelab6

import numpy as np


def point_registration(A, B):
    # Compute Centroids A & B
    A = np.array(A)
    B = np.array(B)
    uA = np.mean(A, axis=1, keepdims=True)
    uB = np.mean(B, axis=1, keepdims=True)


    # Find centered coordinates
    Acen = A - uA
    Bcen = B - uB

    # Calculate H
    H = Acen @ np.transpose(Bcen)

    # SVD
    U, S, Vt = np.linalg.svd(H)
    V = np.transpose(Vt)


    # Calculate R
    R = V @ np.transpose(U)
    if np.linalg.det(R)<0:  # Handle special reflection case
        V[:,2] *= -1
        R = V @ np.transpose(U)
    else:
        None
    
    # Calculate d
    d = uB - R @ uA

    # Create T
    T = np.zeros((4,4), dtype=float)
    T = np.vstack((np.hstack((R,d)),[0, 0, 0, 1]))
    return T

def main():
    np.set_printoptions(precision=3, suppress=1)    # set print precision and suppression

    A = np.array([
        [681.2, 526.9, 914.8],
        [542.3, 381.0, 876.5],
        [701.2, 466.3, 951.4],
        [598.4, 556.8, 876.9],
        [654.3, 489.0, 910.2]]).T

    B = np.array([
        [110.1, 856.3, 917.8],
        [115.1, 654.9, 879.5],
        [167.1, 827.5, 954.4],
        [ 30.4, 818.8, 879.9],
        [117.9, 810.4, 913.2]]).T   
    
    # Call point_registration() and find RMSE
    T = point_registration(A,B)
    R = T[:3, :3]
    d = T[:3, 3].reshape(3, 1)
    rmse = np.sqrt(np.mean(np.sum((R @ A + d - B)**2, axis=0)))
    print("RMSE:")
    print(rmse)
    print("RRt")
    print(R @ np.transpose(R))
    print("Determinant of R")
    print(np.linalg.det(R))



if __name__ == "__main__":
    main()