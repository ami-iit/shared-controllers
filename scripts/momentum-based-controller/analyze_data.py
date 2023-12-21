from wholebodycontrollib import loggerplotterlib
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse

logger = loggerplotterlib.LoggerPlotter()


def add_offset(a, b):
    return a + b

# LOAD DATA
# Check if a filename was passed
parser = argparse.ArgumentParser()
parser.add_argument('filename', nargs='?')
args = parser.parse_args()
filename = args.filename

log_directory = "data/"

if filename is None:
    # Otherwise search for the latest file
    files = os.listdir(log_directory)
    filtered_files = [file for file in files if file.startswith("logged_data_") and file.endswith(".pickle")]
    sorted_files = sorted(filtered_files, key=lambda x: os.path.getmtime(os.path.join(log_directory, x)), reverse=True)
    filename = sorted_files[0]

# Load Data
logger.load_data_from_file(log_directory + filename)
print("Loading data from '" + log_directory + filename + "'")

# Set time zero
logger.compute_new_variable(['t'], 't_0', add_offset, -1)


# Tracking
fig = logger.plot_data('t_0', ['p_com', 'p_com_des'], 'CoM position tracking', y_data_labels=['meas', 'des'], show_plot=False, sub_titles=["x", "y", "z"], x_label="Time [s]", y_label="Position [m]",  x_lim=[0, 35])
fig = logger.plot_data('t_0', ['v_com', 'v_com_des'], 'CoM velocity tracking', y_data_labels=['meas', 'des'], show_plot=False, sub_titles=["x", "y", "z"], x_label="Time [s]", x_lim=[0, 20])
fig = logger.plot_data('t_0', ['ang_mom', 'ang_mom_des'], 'Angular Momentum tracking', y_data_labels=['meas', 'des'], show_plot=False, sub_titles=["x", "y", "z"], x_label="Time [s]", x_lim=[0, 35])
fig = logger.plot_data('t_0', ['joint_pos', 'joint_pos_des'], 'Joint Position', single_plot=True, y_data_labels=['meas', 'des'], show_plot=False, selected_variables = [0, 1, 2, 8, 9, 11], sub_titles=["Left Shoulder Pitch", "Left Shoulder Roll", "Left Shoulder Yaw", "Left Hip Pitch", "Left Hip Roll", "Left Knee"], x_label="Time [s]", y_label="Angle [rad]", x_lim=[0, 35])

# When plotting the joint torques remember that the tau vector is smaller then the joint_pos vector because of the selcted joints
fig = logger.plot_data('t_0', ['tau'], 'Joint Torque', show_plot=False, selected_variables = [0, 1, 2], sub_titles=["Left Shoulder Pitch", "Left Shoulder Roll", "Left Shoulder Yaw"], x_label="Time [s]", y_label="Torque [N m]", x_lim=[0, 35])  
fig = logger.plot_data('t_0', ['tau'], '', show_plot=False, selected_variables = [6, 7, 9], sub_titles=["Left Hip Pitch", "Left Hip Roll", "Left Knee"], x_label="Time [s]", y_label="Torque [N m]", x_lim=[0, 35])  

# Torques
logger.compute_new_variable(['tau'], 'tau_norm', operator=np.linalg.norm)
fig = logger.plot_data('t_0', ['tau_norm'], 'Joint Torque Norm', show_plot=False, x_label="Time [s]", y_label="Torque [N m]", x_lim=[0, 35])

plt.show()
