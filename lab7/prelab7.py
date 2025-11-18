# EE471 Prelab 7 11/10/2025
# William Taing wtaing@calpoly.edu
# Description: for Prelab7 7.0. PID Controller Implementation for Visual Servoing

import numpy as np
import matplotlib.pyplot as plt


class PIDController:
    def __init__(self, dim=3, dt=0.05):
        # Initialize gains (tuned for position control in mm)
        self.Kp = 0.5 * np.eye(dim) # Proportional gain
        self.Ki = 0.05 * np.eye(dim) # Integral gain
        self.Kd = 0.1 * np.eye(dim) # Derivative gain
        # Initialize error terms
        self.error_integral = np.zeros(dim)
        self.error_prev = np.zeros(dim)
        self.dt = dt # Control period in seconds

    def compute_pid(self, error):
        # Initialize u
        u = np.zeros(3)

        # Accumulate Error
        self.error_integral += error * self.dt

        # Calculate u
        u = (self.Kp @ error) + (self.Ki @ self.error_integral) + (self.Kd @ ((error - self.error_prev) / self.dt))

        # Save error for next dt
        self.error_prev = error

        return u

def test():
    # Initialize controller with 50ms control period
    controller = PIDController(dim=3, dt=0.05)

    # Time vector (5 seconds of simulation)
    t = np.linspace(0, 5, 801)

    # Initialize arrays to store results
    errors = np.zeros((len(t), 3))
    outputs = np.zeros((len(t), 3))

    # Set initial error (different for each axis to test independently)
    error = np.array([10., 8., 6.]) # mm

    # Simulate system response
    for i in range(len(t)):
        # Store current error
        errors[i] = error
        # Compute PID control output (desired velocity in mm/s)
        outputs[i] = controller.compute_pid(error)
        # Simple model: error reduces due to control action
        # Error reduction = velocity * timestep
        error = error - outputs[i] * 0.05
    


    # Error Vs Time Plot

    # Convert lists to arrays
    t_list = np.array(t)
    error_list = np.array(errors)

    # Slice x, y, z error
    ex = error_list[:,0]
    ey = error_list[:,1]
    ez = error_list[:,2]
    
    # End-effector pose vs time 
    fig, axs = plt.subplots(3)    # create subplots
    # Plot x error
    axs[0].plot(t_list, ex, linestyle='-')
    axs[0].set_title("X error vs time")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Error (mm)")
    # Plot y error
    axs[1].plot(t_list, ey, linestyle='--')
    axs[1].set_title("Y error vs time")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Error (mm)")
    # Plot z error
    axs[2].plot(t_list, ez, linestyle='-.')
    axs[2].set_title("Z error vs time")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Error (mm)")
    # Plot settings
    fig.suptitle("error vs time")
    plt.tight_layout()



    # Control Output vs Time Plot

    # Convert lists to arrays
    t_list = np.array(t)
    outputs_list = np.array(outputs)
    
    # Slice x, y, z control
    cx = outputs_list[:,0]
    cy = outputs_list[:,1]
    cz = outputs_list[:,2]

    fig, axs = plt.subplots(3)    # create subplots
    # Plot x error
    axs[0].plot(t_list, cx, linestyle='-')
    axs[0].set_title("X control vs time")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Control (mm/s)")
    # Plot y error
    axs[1].plot(t_list, cy, linestyle='--')
    axs[1].set_title("Y control vs time")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Control (mm/s)")
    # Plot z error
    axs[2].plot(t_list, cz, linestyle='-.')
    axs[2].set_title("Z control vs time")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Control (mm/s)")
    # Plots Settings
    fig.suptitle("control output vs time")
    plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    test()