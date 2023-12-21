from wholebodycontrollib import wholebodycontrol
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



def compute_joint_torque(H_b, s, model, postural_task_controller, momentum_controller, f_hands_des, Adeq_local, bdeq_local, feet_frame, hands_frame):
    
    model.set_state(H_b, s, np.zeros(6), 0 * s)

    J_feet = model.get_frames_jacobian(feet_frame)
    J_hands = model.get_frames_jacobian(hands_frame)

    Jdot_nu_feet = model.get_frames_bias_acceleration(feet_frame)    
    w_H_frames = model.get_frames_transform(feet_frame + hands_frame)

    M = model.get_mass_matrix()
    h = model.get_generalized_bias_force()
    Jcm = model.get_centroidal_momentum_jacobian()
    H = model.get_centroidal_momentum()
    p_com = model.get_center_of_mass_position()
    v_com = model.get_center_of_mass_velocity()
    B =  np.block([[np.zeros([6, len(s)])], [np.eye(len(s))]])

    Jf = np.vstack((J_feet,J_hands))
    Jc = J_feet
    Jdot_nu = Jdot_nu_feet

    postural_task_controller.set_desired_posture(s, 0*s)
    momentum_controller.set_desired_center_of_mass_trajectory(p_com, v_com, np.zeros(3))
    momentum_controller.set_desired_angular_momentum(np.zeros(3))

    [tau_0_model, tau_0_sigma] = postural_task_controller.get_postural_task_torque(s, 0*s, M, Jf, h)
    [Adeq_local_left_foot, _] = momentum_controller.tranform_local_wrench_task_into_global(Adeq_local, bdeq_local, w_H_frames[:4,:])
    [Adeq_local_right_foot, _] = momentum_controller.tranform_local_wrench_task_into_global(Adeq_local, bdeq_local, w_H_frames[4:8,:])
    Adeq = np.block([[Adeq_local_left_foot                         , np.zeros([Adeq_local_left_foot.shape[0], 6])],
                        [np.zeros([Adeq_local_right_foot.shape[0], 6]), Adeq_local_right_foot]])
    bdeq = np.block([bdeq_local, bdeq_local])

    [Aeq, beq] = momentum_controller.get_momentum_control_tasks(H, p_com, w_H_frames)

    Aeq_aug = np.block([[Aeq],
                        [np.zeros([6,12]),  np.eye(6),     np.zeros([6,6])],
                        [np.zeros([6,12]),  np.zeros([6,6]), np.eye(6)]]) 
    beq_aug = np.concatenate((beq, f_hands_des))
    Adeq_aug = np.block([Adeq,  np.zeros([Adeq.shape[0], 12])])

    [tau_sigma, tau_model] = wholebodycontrol.get_torques_projected_dynamics(tau_0_model, tau_0_sigma, Jc, Jf, Jdot_nu, M, h, B)
    wrench_qp = wholebodycontrol.WrenchQP()
    f = wrench_qp.solve(tau_model, tau_sigma, Aeq_aug, beq_aug, None, None, 1.0, "quadprog")

    if f is None:
        print("Failed to solve the optimization")
    return  tau_sigma @ f + tau_model

def add_offset(a, b):
    return a + b

# load model
human_model = wholebodylib.robot(human_configuration.urdf_path, human_configuration.joints_list, human_configuration.base_link)

## initialize controllers gains (all the gains are set to zero)
# postural controller
postural_task_controller = wholebodycontrol.PosturalTaskController(len(human_configuration.joints_list))
postural_task_gain = wholebodycontrol.PosturalTaskGain(len(human_configuration.joints_list))
Kp = np.zeros(len(human_configuration.joints_list))
postural_task_gain.Kp = np.diag(Kp)
Kd = Kp
postural_task_gain.Kd = np.diag(Kd)
postural_task_controller.set_gain(postural_task_gain)

# momentum controller 
momentum_controller = wholebodycontrol.MomentumController(human_model.kindyn.model().getTotalMass())
momentum_controller_gain = wholebodycontrol.MomentumControllerGain()
momentum_controller_gain.Ki = np.zeros(6)
momentum_controller_gain.Kp = np.zeros(6)
momentum_controller.set_gain(momentum_controller_gain)
momentum_controller.set_desired_center_of_mass_trajectory(np.zeros(3), np.zeros(3), np.zeros(3))

# hand wrench
load = 2
f_l_hand_des = np.array([0,  35, -9.81 * load /2 , 0, 0, 0])
f_r_hand_des = np.array([0, -35, -9.81 * load /2, 0, 0, 0])
f_hand_des = np.concatenate((f_l_hand_des, f_r_hand_des))

# contact constraint
[Adeq_local, bdeq_local] = wholebodycontrol.MomentumController.get_rigid_contact_contraint(1/3, 1/75, np.array([[-0.12, 0.12], [-0.05, 0.05]]), 10)

# compute variable
logger.compute_new_variable(['H_human', 'joint_pos_human'], 'tau_human', compute_joint_torque, human_model, postural_task_controller, momentum_controller, f_hand_des, Adeq_local, bdeq_local, ["LeftToe", "RightToe"], ["LeftHand", "RightHand"])
logger.compute_new_variable(['t'], 't_0', add_offset, -6)
logger.compute_new_variable(['tau_human'], 'tau_human_norm', operator=np.linalg.norm)

# plot Human
fig = logger.plot_data('t_0', ['tau_human_norm'], 'Joint Torque Norm', show_plot=False, x_label="Time [s]", y_label="Torque [N m]", x_lim=[0, 20])
fig = logger.plot_data('t_0', ['tau_human'], 'Joint Torque Norm', show_plot=False, x_label="Time [s]", y_label="Torque [N m]", selected_variables = [30, 8, 22], sub_titles=["L5S1", "Shoulder Pitch", "Hip Pitch"], x_lim=[0, 20])

# plot human and robot together
fig = logger.plot_data('t_0', ['tau_human', 'tau'], 'Joint Torque', single_plot=True, y_data_labels=['Human', 'Robot'], show_plot=False, x_label="Time [s]", y_label="Torque [N m]", selected_variables = [[30, 8, 22], [9, 0, 6]], sub_titles=[["L5S1", "Shoulder Pitch", "Hip Pitch"], ["Knee", "Shoulder Pitch", "Hip Pitch"]], x_lim=[0, 15], fill_x_upper_treshold=7.92, fill_x_upper_color='mistyrose')

plt.show()
