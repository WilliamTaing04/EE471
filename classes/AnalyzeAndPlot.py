import numpy as np
import matplotlib.pyplot as plt

def plot_angle_list(t_list,a_list,title):
    # Convert list to np array
    t_array = np.array(t_list)
    angles_array = np.array(a_list)

    fig, axs = plt.subplots(2,2)    # create subplots
    # Plot motor 1
    axs[0,0].plot(t_array, angles_array[:,0])
    axs[0,0].set_title("Motor 1")
    # Plot motor 2
    axs[0,1].plot(t_array, angles_array[:,1])
    axs[0,1].set_title("Motor 2")
    # Plot motor 3
    axs[1,0].plot(t_array, angles_array[:,2])
    axs[1,0].set_title("Motor 3")
    # Plot motor 4
    axs[1,1].plot(t_array, angles_array[:,3])
    axs[1,1].set_title("Motor 4")
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

    # Calculate some basic statistics
    avg_dt = np.mean(delta_t)
    max_dt = np.max(delta_t)
    min_dt = np.min(delta_t)
    q1=np.percentile(delta_t, 25)
    q3=np.percentile(delta_t, 75)

    # Create focused_dt within IQR
    focused_dt=[x for x in delta_t if (q1-1.5*(q3-q1)) <= x <= (q3+1.5*(q3-q1))]

    # Plot histogram
    plt.figure(figsize=(8, 5))
    plt.hist(delta_t, bins=100)
    plt.title(title)
    plt.xlabel("Sample Time (S)")
    plt.ylabel("Frequency")
    plt.show()

    # Plot histogram
    plt.figure(figsize=(8, 5))
    plt.hist(focused_dt, bins=100)
    plt.title(title+" Focused IQR")
    plt.xlabel("Sample Time (S)")
    plt.ylabel("Frequency")
    plt.show()
