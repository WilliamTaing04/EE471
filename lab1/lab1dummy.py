import pickle
import classes.Robot as robot

def save_to_pickle(data: dict, filename: str):
    with open(filename, "wb") as f:
        pickle.dump(data, f)

def load_from_pickle(filename: str):
    with open(filename, "rb") as f:
        return pickle.load(f)


# Load data from pickle file 10s
data10 = load_from_pickle("lab1_data_10s.pk1")
t_pickle10 = data10["timestamps_s"]
q_pickle10 = data10["joint_deg"]
# Load data from pickle file 10s
data2 = load_from_pickle("lab1_data_2s.pk1")
t_pickle2 = data2["timestamps_s"]
q_pickle2 = data2["joint_deg"]
# Create plots for 10s
robot.plot_angle_list(t_pickle10,q_pickle10,"Motor Angle vs Time: 10s Traj Time")
robot.plot_samplet_hist(t_pickle10,"Distribution of Sample Intervals: 10s Traj Time")
# Create plots for 10s
robot.plot_angle_list(t_pickle2,q_pickle2,"Motor Angle vs Time: 2s Traj Time")
robot.plot_samplet_hist(t_pickle2,"Distribution of Sample Intervals: 2s Traj Time")

