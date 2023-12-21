from wholebodycontrollib import loggerplotterlib
from wholebodycontrollib import wholebodylib

import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import argparse

sys.path.append('..')
import robots.ergoCubSN000.configuration as robot_configuration
import robots.human.configuration as human_configuration

logger = loggerplotterlib.LoggerPlotter()


## LOAD DATA
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

## LOAD MODELS
# Open kinDyn objects
robot_model = wholebodylib.robot(robot_configuration.urdf_path, robot_configuration.joints_list, robot_configuration.base_link)
human_model = wholebodylib.robot(human_configuration.urdf_path, human_configuration.joints_list, human_configuration.base_link)

def forward_kinematics(H_b, s, model, frame_name):
    model.set_state(H_b, s, np.zeros((6,)), s)
    return model.get_frame_transform(frame_name)

def get_position_from_pose(H, x_idx, y_idx):
    return H[x_idx, y_idx]

def scale_position(z):
    return z + 0.5*(z-0.85)

def add_offset(a, b):
    return a + b

# Set time zero
logger.compute_new_variable(['t'], 't_0', add_offset, -6)

# Forward Kinematics
logger.compute_new_variable(['H_robot', 'joint_pos'], 'H_robot_hand', forward_kinematics, robot_model, 'r_hand_palm')
logger.compute_new_variable(['H_human', 'joint_pos_human'], 'H_human_hand', forward_kinematics, human_model, 'LeftHand')

fig = logger.plot_data('t_0', ['H_robot_hand', 'H_human_hand'], 'Hand Height', show_plot=False, selected_variables = [(2, 3)])

# Extract z and scale
logger.compute_new_variable(['H_robot_hand'], 'z_robot_hand', get_position_from_pose, 2, 3)
logger.compute_new_variable(['H_human_hand'], 'z_human_hand', get_position_from_pose, 2, 3)
logger.compute_new_variable(['z_human_hand'], 'z_human_hand_scaled', scale_position)
logger.compute_new_variable(['z_human_hand'], 'z_human_hand_offset', add_offset, logger.data["z_robot_hand"][50] - logger.data["z_human_hand"][50])

fig = logger.plot_data('t_0', ['z_robot_hand', 'z_human_hand_scaled'], 'Hand Height', show_plot=False, x_label="Time [s]", y_label="Height [m]", x_lim=[0, 15])

fig = logger.plot_data('t_0', ['z_robot_hand', 'z_human_hand_offset'], 'Hand Height', show_plot=False, x_label="Time [s]", y_label="Height [m]", x_lim=[0, 15])

plt.show()
