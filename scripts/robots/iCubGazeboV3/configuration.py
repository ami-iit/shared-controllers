import numpy as np
import resolve_robotics_uri_py

# Model and ControlBoard
robot_name = "icubSim"
urdf_path = str(resolve_robotics_uri_py.resolve_robotics_uri("package://iCub/robots/iCubGazeboV3/model.urdf"))
base_link = "root_link"

joints_list = ['torso_pitch', 
               'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow',
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
    'torso_pitch'      : 'positionDirect', 
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

## Hands 35cm 
state_machine_configurations['hands_35'] = {}
state_machine_configurations['hands_35']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch' : -0.89855995,
    'l_shoulder_roll'  :  0.40991316, 
    'l_shoulder_yaw'   : -0.47587623, 
    'l_elbow'          :  0.54306246,
    'r_shoulder_pitch' : -0.89855995,
    'r_shoulder_roll'  :  0.40991316,
    'r_shoulder_yaw'   : -0.47587623,
    'r_elbow'          :  0.54306246,
    'l_hip_pitch'      :  2.00712864,
    'l_hip_roll'       :  0.05210082, 
    'l_hip_yaw'        : -0.05780139, 
    'l_knee'           : -1.22173046, 
    'l_ankle_pitch'    : -0.38574655, 
    'l_ankle_roll'     : -0.07867542,
    'r_hip_pitch'      :  2.00712864,
    'r_hip_roll'       :  0.05210082,
    'r_hip_yaw'        : -0.05780139,
    'r_knee'           : -1.22173046,
    'r_ankle_pitch'    : -0.38574655,
    'r_ankle_roll'     : -0.07867542
    }
state_machine_configurations['hands_35']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 70cm 0.20742222  0.06424979 -0.00483591 -0.1410721  -0.0974814  -0.06378178
state_machine_configurations['hands_75'] = {}
state_machine_configurations['hands_75']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch' : -0.46376328,
    'l_shoulder_roll'  :  0.2474646 , 
    'l_shoulder_yaw'   : -0.47479289, 
    'l_elbow'          :  0.44648703,
    'r_shoulder_pitch' : -0.46376328,
    'r_shoulder_roll'  :  0.2474646 ,
    'r_shoulder_yaw'   : -0.47479289,
    'r_elbow'          :  0.44648703,
    'l_hip_pitch'      :  0.20742222,
    'l_hip_roll'       :  0.06424979, 
    'l_hip_yaw'        : -0.00483591, 
    'l_knee'           : -0.1410721 , 
    'l_ankle_pitch'    : -0.0974814 , 
    'l_ankle_roll'     : -0.06378178,
    'r_hip_pitch'      :  0.20742222,
    'r_hip_roll'       :  0.06424979,
    'r_hip_yaw'        : -0.00483591,
    'r_knee'           : -0.1410721 ,
    'r_ankle_pitch'    : -0.0974814 ,
    'r_ankle_roll'     : -0.06378178
    }
state_machine_configurations['hands_75']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}
