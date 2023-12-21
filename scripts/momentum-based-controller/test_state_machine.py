from wholebodycontrollib import statemachine
from wholebodycontrollib import visualizer
from wholebodycontrollib import wholebodylib
from wholebodycontrollib import robotInterface

from utils import configuration_hadler

import time
import sys
import numpy as np

sys.path.append('..')
import robots.ergoCubSN000.configuration as robot_configuration
import robots.human.configuration as human_configuration

# Parameters
controller_frequency = 0.003 # seconds
hands_tracking_gain = 10.0


# Load Model
model = wholebodylib.robot(robot_configuration.urdf_path, robot_configuration.joints_list, robot_configuration.base_link)

# Initialize state machine
state_machine = statemachine.StateMachine(repeat=False)


# Lifting Configurations
configurations = configuration_hadler.statemachine_configurations_generator(robot_configuration, model, ["hands_70", "hands_70", "hands_100"], [1 ,1, 100])


# Connect with human
human_model = wholebodylib.robot(human_configuration.urdf_path, human_configuration.joints_list, human_configuration.base_link)
human_robot_interface = robotInterface.robotInterface(human_configuration.robot_name, "/local", human_configuration.joints_list, human_configuration.remote_control_board_list)
human_robot_interface.open()

# Initialize visualizer
visualizer = visualizer.Visualizer()
visualizer.add_model("Robot", model, robot_configuration.urdf_path, robot_configuration.base_link)
visualizer.add_model("Human", human_model, human_configuration.urdf_path, human_configuration.base_link)
visualizer.run()

# reset clock
time_prev = time.time()
t = 0

for configuration in configurations:
    state_machine.add_configuration(configuration)

while True:

    if not state_machine.update(time.time()):
        break

    # Kinematics
    w_H_r_hand = model.get_frames_transform(["r_hand_palm"])
    w_H_l_hand = model.get_frames_transform(["l_hand_palm"])
    J_r_hand = model.get_frames_jacobian(["r_hand_palm"])

    # Human
    human_s = human_robot_interface.get_joints_position()
    human_ds = human_robot_interface.get_joints_velocity()
    human_feet = np.array( [[-1, 0, 0, 1], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]] )
    human_base_pose = human_model.get_base_pose_from_contacts(human_s, {'LeftToe' : human_feet, 'RightToe' : human_feet})
    human_w_b = human_model.get_base_velocity_from_contacts(human_base_pose, human_s, human_ds, ["LeftHand", "RightHand"])
    human_model.set_state(human_base_pose, human_s, human_w_b, human_ds)
    human_w_H_frames = human_model.get_frames_transform(["LeftToe", "RightToe", "LeftHand", "RightHand"])


    # Get state machine output
    joint_pos_des, joint_vel_des, joint_acc_des, _, _, _ = state_machine.get_state(True, human_w_H_frames[10,3], w_H_r_hand[2,3], J_r_hand, controller_frequency, hands_tracking_gain)
    base_pose_des = model.get_base_pose_from_contacts(joint_pos_des, {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)})
    w_b_des = model.get_base_velocity_from_contacts(base_pose_des, joint_pos_des, joint_vel_des, ["l_sole", "r_sole"])

    # Update visualizer
    model.set_state(base_pose_des, joint_pos_des, w_b_des, joint_vel_des)
    visualizer.update_model("Robot", base_pose_des, joint_pos_des)
    visualizer.update_model("Human", human_base_pose, human_s)


    # wait and update timer
    dt = time.time() - time_prev
    if dt < controller_frequency:
        time.sleep((controller_frequency - dt))
    else:
        print("Deadline miss: " + str(dt) + " [s]")

    dt = time.time() - time_prev
    t = t+dt

    time_prev = time.time()

