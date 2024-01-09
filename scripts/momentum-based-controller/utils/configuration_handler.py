from wholebodycontrollib import statemachine
from wholebodycontrollib import wholebodylib

import numpy as np

def statemachine_configurations_generator(robot_configuration, model, configuration_names, durations) :
    
    configurations = []
    for i, configuration_name in enumerate(configuration_names):
        state = robot_configuration.state_machine_configurations[configuration_name]
        joint_pos = np.array([state['joints_position'][joint_name] for joint_name in robot_configuration.joints_list])
        base_pose = model.get_base_pose_from_contacts(joint_pos, state['contacts'])
        model.set_state(base_pose, joint_pos, np.zeros(6), np.zeros(joint_pos.shape))
        p_com = model.get_center_of_mass_position()

        configurations.append(statemachine.Configuration(joint_pos, p_com, durations[i]))

    return configurations