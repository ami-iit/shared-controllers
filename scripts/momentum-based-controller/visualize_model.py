from wholebodycontrollib import loggerplotterlib
from wholebodycontrollib import wholebodylib
from wholebodycontrollib import visualizer


import resolve_robotics_uri_py
import idyntree.bindings as iDynTree
import numpy as np
import os
import sys
import time

sys.path.append('..')
import robots.ergoCubSN000.configuration as robot_configuration
import robots.human.configuration as human_configuration


## LOAD DATA
logger = loggerplotterlib.LoggerPlotter()

# Search for the latest file
log_directory = "data/"
files = os.listdir(log_directory)
filtered_files = [file for file in files if file.startswith("logged_data_") and file.endswith(".pickle")]
sorted_files = sorted(filtered_files, key=lambda x: os.path.getmtime(os.path.join(log_directory, x)), reverse=True)

# Load Data
logger.load_data_from_file(log_directory + sorted_files[0])
print("Loading data from '" + log_directory + sorted_files[0] + "'")


## PREPARE VISUALIZER
visualizer = visualizer.Visualizer()

# Open kinDyn objects
model = wholebodylib.robot(robot_configuration.urdf_path, robot_configuration.joints_list, robot_configuration.base_link)
human_model = wholebodylib.robot(human_configuration.urdf_path, human_configuration.joints_list, human_configuration.base_link)

# Add models to visualizer
visualizer.add_model("Robot", model, robot_configuration.urdf_path, robot_configuration.base_link)
visualizer.add_model("Desired", model, robot_configuration.urdf_path, robot_configuration.base_link, [0.8, 0.0, 0.0, 0.1])
visualizer.add_model("Human", human_model, human_configuration.urdf_path, human_configuration.base_link)

visualizer.run()

n_data_points = len(logger.data['H_robot'])

# RUN VISUALIZER
time_prev = time.time()
for idx in range(n_data_points):

    visualizer.update_model("Robot", logger.data['H_robot'][idx], logger.data['joint_pos'][idx])
    visualizer.update_model("Desired", logger.data['H_robot'][idx], logger.data['joint_pos_des'][idx])
    visualizer.update_model("Human", logger.data['H_human'][idx], logger.data['joint_pos_human'][idx])

    visualizer.draw()

    if idx+1 < n_data_points:
        dt = time.time() - time_prev
        dt_data = logger.data['t'][idx+1] - logger.data['t'][idx+1]
        if dt < dt_data :
            time.sleep(dt_data - dt)
        time_prev = time.time()

visualizer.close()


