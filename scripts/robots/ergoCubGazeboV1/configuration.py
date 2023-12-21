import numpy as np
import resolve_robotics_uri_py

# Model and ControlBoard
robot_name = "ergocubSim"
urdf_path = str(resolve_robotics_uri_py.resolve_robotics_uri("package://ergoCub/robots/ergoCubGazeboV1/model.urdf"))
base_link = "root_link"

joints_list = ['l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow',
               'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow', 
               'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll',
               'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll']

remote_control_board_list =  ["/" + robot_name + "/torso", 
                              "/" + robot_name + "/left_arm", 
                              "/" + robot_name + "/right_arm", 
                              "/" + robot_name + "/left_leg", 
                              "/" + robot_name + "/right_leg"]

# Controller Configuration

## Control Mode
joints_control_mode = {
    'l_shoulder_pitch' : 'torque', 
    'l_shoulder_roll'  : 'torque', 
    'l_shoulder_yaw'   : 'torque', 
    'l_elbow'          : 'torque',
    'r_shoulder_pitch' : 'torque', 
    'r_shoulder_roll'  : 'torque', 
    'r_shoulder_yaw'   : 'torque', 
    'r_elbow'          : 'torque', 
    'l_hip_pitch'      : 'torque', 
    'l_hip_roll'       : 'torque', 
    'l_hip_yaw'        : 'torque', 
    'l_knee'           : 'torque', 
    'l_ankle_pitch'    : 'torque', 
    'l_ankle_roll'     : 'torque',
    'r_hip_pitch'      : 'torque', 
    'r_hip_roll'       : 'torque', 
    'r_hip_yaw'        : 'torque', 
    'r_knee'           : 'torque', 
    'r_ankle_pitch'    : 'torque', 
    'r_ankle_roll'     : 'torque'
}

## Gains
controller_gains = {}
controller_gains['postural_task'] = {}
controller_gains['momentum_task'] = {}

controller_gains['momentum_task']['Ki'] = [50, 50, 200, 0, 0, 0]
controller_gains['momentum_task']['Kp'] = [1.5, 1.5, 2, 0.025, 0.025, 0.025]

controller_gains['postural_task']['Kp'] = {
    'torso_pitch'      :   0.0,
    'l_shoulder_pitch' : 195.0,
    'l_shoulder_roll'  : 120.0, 
    'l_shoulder_yaw'   :  30.0, 
    'l_elbow'          :  90.0,
    'r_shoulder_pitch' : 195.0,
    'r_shoulder_roll'  : 120.0,
    'r_shoulder_yaw'   :  30.0,
    'r_elbow'          :  90.0,
    'l_hip_pitch'      : 300.0,
    'l_hip_roll'       :  30.0, 
    'l_hip_yaw'        :  30.0, 
    'l_knee'           : 270.0, 
    'l_ankle_pitch'    : 210.0, 
    'l_ankle_roll'     :  30.0,
    'r_hip_pitch'      : 300.0,
    'r_hip_roll'       :  30.0,
    'r_hip_yaw'        :  30.0,
    'r_knee'           : 270.0,
    'r_ankle_pitch'    : 210.0,
    'r_ankle_roll'     :  30.0
}

Kd_from_Ki = lambda x : x**(1/2) / 10
controller_gains['postural_task']['Kd'] = {}
for joint_name, Kp in controller_gains['postural_task']['Kp'].items():
    controller_gains['postural_task']['Kd'][joint_name] = Kd_from_Ki(Kp)

# State Machine
state_machine_configurations = {}

## Hands 40cm
state_machine_configurations['hands_40'] = {}
state_machine_configurations['hands_40']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch' : -1.0085969 ,
    'l_shoulder_roll'  :  0.27003022, 
    'l_shoulder_yaw'   : -0.21660158, 
    'l_elbow'          :  0.44837469,
    'r_shoulder_pitch' : -1.0085969 ,
    'r_shoulder_roll'  :  0.27003022,
    'r_shoulder_yaw'   : -0.21660158,
    'r_elbow'          :  0.44837469,
    'l_hip_pitch'      :  1.82328961,
    'l_hip_roll'       :  0.03101905, 
    'l_hip_yaw'        : -0.03029954, 
    'l_knee'           : -0.90969047, 
    'l_ankle_pitch'    : -0.17639512, 
    'l_ankle_roll'     : -0.04340851,
    'r_hip_pitch'      :  1.82328961,
    'r_hip_roll'       :  0.03101905,
    'r_hip_yaw'        : -0.03029954,
    'r_knee'           : -0.90969047,
    'r_ankle_pitch'    : -0.17639512,
    'r_ankle_roll'     : -0.04340851
    }
state_machine_configurations['hands_40']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 70cm
state_machine_configurations['hands_70'] = {}
state_machine_configurations['hands_70']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch' : -0.70128084,
    'l_shoulder_roll'  :  0.20943964, 
    'l_shoulder_yaw'   :  0.09320089, 
    'l_elbow'          :  0.37556451,
    'r_shoulder_pitch' : -0.70128084,
    'r_shoulder_roll'  :  0.20943964,
    'r_shoulder_yaw'   :  0.09320089,
    'r_elbow'          :  0.37556451,
    'l_hip_pitch'      :  1.08421339,
    'l_hip_roll'       :  0.03497414, 
    'l_hip_yaw'        : -0.01895839, 
    'l_knee'           : -0.66130679, 
    'l_ankle_pitch'    : -0.19125154, 
    'l_ankle_roll'     : -0.03965493,
    'r_hip_pitch'      :  1.08421339,
    'r_hip_roll'       :  0.03497414,
    'r_hip_yaw'        : -0.01895839,
    'r_knee'           : -0.66130679,
    'r_ankle_pitch'    : -0.19125154,
    'r_ankle_roll'     : -0.03965493
    }
state_machine_configurations['hands_70']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 80cm
state_machine_configurations['hands_80'] = {}
state_machine_configurations['hands_80']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch' : -0.53361366,
    'l_shoulder_roll'  :  0.20943954, 
    'l_shoulder_yaw'   :  0.2925762 , 
    'l_elbow'          :  0.46865939,
    'r_shoulder_pitch' : -0.53361366,
    'r_shoulder_roll'  :  0.20943954,
    'r_shoulder_yaw'   :  0.2925762 ,
    'r_elbow'          :  0.46865939,
    'l_hip_pitch'      :  0.60191056,
    'l_hip_roll'       :  0.03637492, 
    'l_hip_yaw'        : -0.01019002, 
    'l_knee'           : -0.37810874, 
    'l_ankle_pitch'    : -0.12520089, 
    'l_ankle_roll'     : -0.03767629,
    'r_hip_pitch'      :  0.60191056,
    'r_hip_roll'       :  0.03637492,
    'r_hip_yaw'        : -0.01019002,
    'r_knee'           : -0.37810874,
    'r_ankle_pitch'    : -0.12520089,
    'r_ankle_roll'     : -0.03767629
    }
state_machine_configurations['hands_80']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 90cm
state_machine_configurations['hands_90'] = {}
state_machine_configurations['hands_90']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch' : -0.38559652,
    'l_shoulder_roll'  :  0.20943979, 
    'l_shoulder_yaw'   :  0.21048467, 
    'l_elbow'          :  0.69015412,
    'r_shoulder_pitch' : -0.38559652,
    'r_shoulder_roll'  :  0.20943979,
    'r_shoulder_yaw'   :  0.21048467,
    'r_elbow'          :  0.69015412,
    'l_hip_pitch'      :  0.11281021,
    'l_hip_roll'       :  0.0366149 , 
    'l_hip_yaw'        : -0.00292579, 
    'l_knee'           : -0.10215589, 
    'l_ankle_pitch'    : -0.04488861, 
    'l_ankle_roll'     : -0.03673821,
    'r_hip_pitch'      :  0.11281021,
    'r_hip_roll'       :  0.0366149 ,
    'r_hip_yaw'        : -0.00292579,
    'r_knee'           : -0.10215589,
    'r_ankle_pitch'    : -0.04488861,
    'r_ankle_roll'     : -0.03673821
}
state_machine_configurations['hands_90']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 120cm
state_machine_configurations['hands_120'] = {}
state_machine_configurations['hands_120']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch' : -0.71343258,
    'l_shoulder_roll'  :  0.43596700, 
    'l_shoulder_yaw'   : -0.06410050, 
    'l_elbow'          :  1.30899679,
    'r_shoulder_pitch' : -0.71343258,
    'r_shoulder_roll'  :  0.43596700,
    'r_shoulder_yaw'   : -0.06410050,
    'r_elbow'          :  1.30899679,
    'l_hip_pitch'      : -0.15421509,
    'l_hip_roll'       :  0.03659853, 
    'l_hip_yaw'        :  0.00113762, 
    'l_knee'           :  0.04043054, 
    'l_ankle_pitch'    :  0.00874356, 
    'l_ankle_roll'     : -0.03662235,
    'r_hip_pitch'      : -0.15421509,
    'r_hip_roll'       :  0.03659853,
    'r_hip_yaw'        :  0.00113762,
    'r_knee'           :  0.04043054,
    'r_ankle_pitch'    :  0.00874356,
    'r_ankle_roll'     : -0.03662235
    }
state_machine_configurations['hands_120']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}

