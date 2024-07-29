import numpy as np
import resolve_robotics_uri_py

# Model and ControlBoard
robot_name = "ergocub"
urdf_path = str(resolve_robotics_uri_py.resolve_robotics_uri("package://ergoCub/robots/ergoCubSN000/model.urdf"))
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

controller_gains['momentum_task']['Ki'] = [0.0*50, 0.0*50, 200, 0, 0, 0]
controller_gains['momentum_task']['Kp'] = [ 0.0*1.5, 0.0*1.5, 2,  0.025, 0.025, 0.025]

# controller_gains['momentum_task']['Ki'] = [0.0*50, 50, 200, 0, 0, 0]
# controller_gains['momentum_task']['Kp'] = [ 1.5, 1.5, 2,  0.025, 0.025, 0.025]

controller_gains['postural_task']['Kp'] = {
    'torso_pitch'      : 1.5*0.0,
    'l_shoulder_pitch' : 1.5*195.0,
    'l_shoulder_roll'  : 1.5*120.0, 
    'l_shoulder_yaw'   : 1.5*30.0, 
    'l_elbow'          : 1.5*90.0,
    'r_shoulder_pitch' : 1.5*195.0,
    'r_shoulder_roll'  : 1.5*120.0,
    'r_shoulder_yaw'   : 1.5*30.0,
    'r_elbow'          : 1.5*90.0,
    'l_hip_pitch'      : 1.5*300.0,
    'l_hip_roll'       : 1.5*30.0, 
    'l_hip_yaw'        : 1.5*30.0, 
    'l_knee'           : 1.5*400.0, 
    'l_ankle_pitch'    : 1.5*210.0, 
    'l_ankle_roll'     : 1.5*30.0,
    'r_hip_pitch'      : 1.5*300.0,
    'r_hip_roll'       : 1.5*30.0,
    'r_hip_yaw'        : 1.5*30.0,
    'r_knee'           : 1.5*400.0,
    'r_ankle_pitch'    : 1.5*210.0,
    'r_ankle_roll'     : 1.5*30.0
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
    'torso_pitch'      :  0.174533  ,
    'l_shoulder_pitch' : -1.29548676,
    'l_shoulder_roll'  :  0.32256893, 
    'l_shoulder_yaw'   : -0.3968961 , 
    'l_elbow'          :  0.18760447,
    'r_shoulder_pitch' : -1.29548676,
    'r_shoulder_roll'  :  0.32256893,
    'r_shoulder_yaw'   : -0.3968961 ,
    'r_elbow'          :  0.18760447,
    'l_hip_pitch'      :  2.00712866,
    'l_hip_roll'       :  0.03332388, 
    'l_hip_yaw'        : -0.03078797, 
    'l_knee'           : -1.22173049, 
    'l_ankle_pitch'    : -0.47609674, 
    'l_ankle_roll'     : -0.04536506,
    'r_hip_pitch'      :  2.00712866,
    'r_hip_roll'       :  0.03332388,
    'r_hip_yaw'        : -0.03078797,
    'r_knee'           : -1.22173049,
    'r_ankle_pitch'    : -0.47609674,
    'r_ankle_roll'     : -0.04536506
    } 
state_machine_configurations['hands_35']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}

## Initial Configuration 40cm
state_machine_configurations['initial_configuration'] = {}
state_machine_configurations['initial_configuration']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch': -1.22247618,
    'l_shoulder_roll': 0.31904745,
    'l_shoulder_yaw': -0.26993846,
    'l_elbow': 0.44355412,
    'r_shoulder_pitch': -1.22236032,
    'r_shoulder_roll': 0.31904246,
    'r_shoulder_yaw': -0.26993959,
    'r_elbow': 0.44360368,
    'l_hip_pitch': 1.01243659,
    'l_hip_roll': 0.03324466,
    'l_hip_yaw': -0.02649463,
    'l_knee': -0.60438973,
    'l_ankle_pitch': -0.0368456,
    'l_ankle_roll': -0.04232412,
    'r_hip_pitch': 1.01250649,
    'r_hip_roll': 0.03330953,
    'r_hip_yaw': -0.02642762,
    'r_knee': -0.60434935,
    'r_ankle_pitch': -0.0367432,
    'r_ankle_roll': -0.04223599
    }
state_machine_configurations['initial_configuration']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 40cm
state_machine_configurations['hands_40'] = {}
state_machine_configurations['hands_40']['joints_position'] = {
    'l_shoulder_pitch' : -1.0085969 ,
    'l_shoulder_roll'  :  0.27003022, 
    'l_shoulder_yaw'   : -0.01660158, 
    'l_elbow'          :  0.44837469,
    'r_shoulder_pitch' : -1.0085969 ,
    'r_shoulder_roll'  :  0.27003022,
    'r_shoulder_yaw'   : -0.01660158,
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

## Hands 60cm
state_machine_configurations['hands_60'] = {}
state_machine_configurations['hands_60']['joints_position'] = {
'l_shoulder_pitch' : -0.7206308787318503,
'l_shoulder_roll' : 0.20943950056593197,
'l_shoulder_yaw' : 0.3842964832462074,
'l_elbow' : 0.2382519496128083,
'r_shoulder_pitch' : -0.7206308787320044,
'r_shoulder_roll' : 0.20943950056600397,
'r_shoulder_yaw' : 0.38429648324619436,
'r_elbow' : 0.23825194961287882,
'l_hip_pitch' : 1.3792800276816277,
'l_hip_roll' : 0.033051989206596535,
'l_hip_yaw' : -0.03120147404549856,
'l_knee' : -1.221730487562488,
'l_ankle_pitch' : -0.4653683198809577,
'l_ankle_roll' : -0.04544870271491313,
'r_hip_pitch' : 1.3792800276809605,
'r_hip_roll' : 0.03305114431984466,
'r_hip_yaw' : -0.031203060215984867,
'r_knee' : -1.2217299456276234,
'r_ankle_pitch' : -0.4653678304165374,
'r_ankle_roll' : -0.045449767576658286,
    }
state_machine_configurations['hands_60']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 70cm
state_machine_configurations['hands_70'] = {}
state_machine_configurations['hands_70']['joints_position'] = {
'l_shoulder_pitch' : -0.27400768347464494,
'l_shoulder_roll' : 0.15,
'l_shoulder_yaw' : 0.15747522868272929,
'l_elbow' : 0.9999999900816579,
'r_shoulder_pitch' : -0.27400768347464494,
'r_shoulder_roll' : 0.15,
'r_shoulder_yaw' : 0.15747522868272929,
'r_elbow' : 0.9999999900816579,
'l_hip_pitch' : 1.1370334834318874,
'l_hip_roll' : 0.03491175938908793,
'l_hip_yaw' : -0.021906763625992417,
'l_knee' : -0.9055386999888033,
'l_ankle_pitch' : -0.34533822529190755,
'l_ankle_roll' : -0.041213069269424155,
'r_hip_pitch' : 1.1370334834318874,
'r_hip_roll' : 0.03491097862116881,
'r_hip_yaw' : -0.021909373279667202,
'r_knee' : -0.9055379055633415,
'r_ankle_pitch' : -0.3453375220319369,
'r_ankle_roll' : -0.04121440511415846,
    }
state_machine_configurations['hands_70']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}

## Hands 75cm
state_machine_configurations['hands_75'] = {}
state_machine_configurations['hands_75']['joints_position'] = {
    'l_shoulder_pitch' : -0.82414356,
    'l_shoulder_roll'  :  0.15, 
    'l_shoulder_yaw'   : -0.04112388, 
    'l_elbow'          :  0.2516476 ,
    'r_shoulder_pitch' : -0.82414356,
    'r_shoulder_roll'  :  0.15,
    'r_shoulder_yaw'   : -0.04112388,
    'r_elbow'          :  0.2516476 ,
    'l_hip_pitch'      :  0.77634033,
    'l_hip_roll'       :  0.03604543, 
    'l_hip_yaw'        : -0.01636604, 
    'l_knee'           : -0.74711944, 
    'l_ankle_pitch'    : -0.32105774, 
    'l_ankle_roll'     : -0.03958513,
    'r_hip_pitch'      :  0.77634033,
    'r_hip_roll'       :  0.03604543,
    'r_hip_yaw'        : -0.01636604,
    'r_knee'           : -0.74711944,
    'r_ankle_pitch'    : -0.32105774,
    'r_ankle_roll'     : -0.03958513
    }
state_machine_configurations['hands_75']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 80cm
state_machine_configurations['hands_80'] = {}
state_machine_configurations['hands_80']['joints_position'] = {
'l_shoulder_pitch' : -0.16867708610992735,
'l_shoulder_roll' : 0.15,
'l_shoulder_yaw' : 0.15240753474043112,
'l_elbow' : 0.9999999901030735,
'r_shoulder_pitch' : -0.16867708610992735,
'r_shoulder_roll' : 0.15,
'r_shoulder_yaw' : 0.15240753474043112,
'r_elbow' : 0.9999999901030735,
'l_hip_pitch' : 0.6422886470715323,
'l_hip_roll' : 0.03637045931016939,
'l_hip_yaw' : -0.00990011530026214,
'l_knee' : -0.4268726740586912,
'l_ankle_pitch' : -0.1611143086891411,
'l_ankle_roll' : -0.03769291851345168,
'r_hip_pitch' : 0.6422886470715323,
'r_hip_roll' : 0.03637095136201407,
'r_hip_yaw' : -0.009908920733000412,
'r_knee' : -0.42687021564613514,
'r_ankle_pitch' : -0.1611121706933918,
'r_ankle_roll' : -0.03769632271926299,
    }
state_machine_configurations['hands_80']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 90cm
state_machine_configurations['hands_90'] = {}
state_machine_configurations['hands_90']['joints_position'] = {
'l_shoulder_pitch' : -0.6376678757464539,
'l_shoulder_roll' : 0.15,
'l_shoulder_yaw' : 0.48984711329822567,
'l_elbow' : 0.2945866256902835,
'r_shoulder_pitch' : -0.6376678757464539,
'r_shoulder_roll' : 0.15,
'r_shoulder_yaw' : 0.48984711329822567,
'r_elbow' : 0.2945866256902835,
'l_hip_pitch' : 0.12473127041595525,
'l_hip_roll' : 0.0367136986663529,
'l_hip_yaw' : -0.0020968967367084334,
'l_knee' : -0.1232309295988608,
'l_ankle_pitch' : -0.06649588083839189,
'l_ankle_roll' : -0.0367732709101038,
'r_hip_pitch' : 0.12473127041595525,
'r_hip_roll' : 0.03670469164305151,
'r_hip_yaw' : -0.002074881145289879,
'r_knee' : -0.12323717702880004,
'r_ankle_pitch' : -0.06650131972871193,
'r_ankle_roll' : -0.03676349504686381,
}
state_machine_configurations['hands_90']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}
## Hands 100cm
state_machine_configurations['hands_100'] = {}
state_machine_configurations['hands_100']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch': -1.22247618,
    'l_shoulder_roll': 0.31904745,
    'l_shoulder_yaw': -0.26993846,
    'l_elbow': 0.44355412,
    'r_shoulder_pitch': -1.22236032,
    'r_shoulder_roll': 0.31904246,
    'r_shoulder_yaw': -0.26993959,
    'r_elbow': 0.44360368,
    'l_hip_pitch' : 0.25479256152244334,
    'l_hip_roll' : 0.03672702367638624,
    'l_hip_yaw' : -0.005588860561231716,
    'l_knee' : -0.3000030972191293,
    'l_ankle_pitch' : -0.1489140270381754,
    'l_ankle_roll' : -0.03714938495742498,
    'r_hip_pitch' : 0.2547925615222638,
    'r_hip_roll' : 0.03673014164682717,
    'r_hip_yaw' : -0.005600019603378294,
    'r_knee' : -0.2999999900681039,
    'r_ankle_pitch' : -0.1489113299337882,
    'r_ankle_roll' : -0.03715465179389699,
}
state_machine_configurations['hands_100']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}

## Initial Configuration 40cm
state_machine_configurations['initial_configuration'] = {}
state_machine_configurations['initial_configuration']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch': -1.22247618,
    'l_shoulder_roll': 0.31904745,
    'l_shoulder_yaw': -0.26993846,
    'l_elbow': 0.44355412,
    'r_shoulder_pitch': -1.22247618,
    'r_shoulder_roll': 0.31904745,
    'r_shoulder_yaw': -0.26993846,
    'r_elbow': 0.44355412,
    'l_hip_pitch': 1.01243659,
    'l_hip_roll': 0.03324466,
    'l_hip_yaw': -0.02649463,
    'l_knee': -0.60434935,
    'l_ankle_pitch': -0.0368456,
    'l_ankle_roll': -0.04232412,
    'r_hip_pitch': 1.01243659,
    'r_hip_roll':  0.03324466,
    'r_hip_yaw': -0.02649463,
    'r_knee': -0.60434935,
    'r_ankle_pitch': -0.0368456,
    'r_ankle_roll': -0.04232412
    }
state_machine_configurations['initial_configuration']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


# ## Hands 100cm
# state_machine_configurations['hands_100'] = {}
# state_machine_configurations['hands_100']['joints_position'] = {
# 'l_shoulder_pitch' : -0.565856047796371,
# 'l_shoulder_roll' : 0.1,
# 'l_shoulder_yaw' : -0.27639761159984146,
# 'l_elbow' : 0.9999999903091813,
# 'r_shoulder_pitch' : -0.5658560477966292,
# 'r_shoulder_roll' : 0.15,
# 'r_shoulder_yaw' : -0.27639761159987694,
# 'r_elbow' : 0.9999999903091077,
# 'l_hip_pitch' : 0.25479256152244334,
# 'l_hip_roll' : 0.03672702367638624,
# 'l_hip_yaw' : -0.005588860561231716,
# 'l_knee' : -0.3000030972191293,
# 'l_ankle_pitch' : -0.1489140270381754,
# 'l_ankle_roll' : -0.03714938495742498,
# 'r_hip_pitch' : 0.2547925615222638,
# 'r_hip_roll' : 0.03673014164682717,
# 'r_hip_yaw' : -0.005600019603378294,
# 'r_knee' : -0.2999999900681039,
# 'r_ankle_pitch' : -0.1489113299337882,
# 'r_ankle_roll' : -0.03715465179389699,
# }
# state_machine_configurations['hands_100']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}

## Hands 110cm
state_machine_configurations['hands_110'] = {}
state_machine_configurations['hands_110']['joints_position'] = {
'l_shoulder_pitch' : -1.187593136260277,
'l_shoulder_roll' : 0.15,
'l_shoulder_yaw' : 1.0770953366556861,
'l_elbow' : 0.18193368790403175,
'r_shoulder_pitch' : -1.187593136260277,
'r_shoulder_roll' : 0.15,
'r_shoulder_yaw' : 1.0770953366556861,
'r_elbow' : 0.18193368790403175,
'l_hip_pitch' : 0.012653776577141817,
'l_hip_roll' : 0.03670074011851012,
'l_hip_yaw' : -0.001140757395108967,
'l_knee' : -0.08054589850165804,
'l_ankle_pitch' : -0.0495485237243446,
'l_ankle_roll' : -0.03671831184555891,
'r_hip_pitch' : 0.012653776577141817,
'r_hip_roll' : 0.036697922806989434,
'r_hip_yaw' : -0.0011361664294649977,
'r_knee' : -0.08054720654492221,
'r_ankle_pitch' : -0.049549663206447866,
'r_ankle_roll' : -0.036715643335976364,
}
state_machine_configurations['hands_110']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}


## Hands 120cm
state_machine_configurations['hands_120'] = {}
state_machine_configurations['hands_120']['joints_position'] = {
'l_shoulder_pitch' : -1.4672207757524425,
'l_shoulder_roll' : 0.15,
'l_shoulder_yaw' : 1.3962633382921588,
'l_elbow' : 0.20694809699717137,
'r_shoulder_pitch' : -1.4672207757524425,
'r_shoulder_roll' : 0.15,
'r_shoulder_yaw' : 1.3962633382921588,
'r_elbow' : 0.20694809699717137,
'l_hip_pitch' : 0.01133947739717035,
'l_hip_roll' : 0.03669771080332698,
'l_hip_yaw' : -0.001116840672047532,
'l_knee' : -0.07809011004338137,
'l_ankle_pitch' : -0.047672366686571876,
'l_ankle_roll' : -0.03671469877865741,
'r_hip_pitch' : 0.01133947739717035,
'r_hip_roll' : 0.036697889629514494,
'r_hip_yaw' : -0.0011173794518901184,
'r_knee' : -0.07808995529251916,
'r_ankle_pitch' : -0.04767223171663766,
'r_ankle_roll' : -0.03671488423403822,
    }
state_machine_configurations['hands_120']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}



## Hands 100cm
state_machine_configurations['hands_final'] = {}
state_machine_configurations['hands_final']['joints_position'] = {
    'torso_pitch'      :  0.0       ,
    'l_shoulder_pitch': -1.5,
    'l_shoulder_roll': 0.31904745,
    'l_shoulder_yaw': -0.26993846,
    'l_elbow': 0.44355412,
    'r_shoulder_pitch': -1.5,
    'r_shoulder_roll': 0.31904745,
    'r_shoulder_yaw': -0.26993846,
    'r_elbow': 0.44355412,
    'l_hip_pitch' : 0.25479256152244334,
    'l_hip_roll' : 0.03672702367638624,
    'l_hip_yaw' : -0.005588860561231716,
    'l_knee' : -0.3000030972191293,
    'l_ankle_pitch' : -0.1489140270381754,
    'l_ankle_roll' : -0.03714938495742498,
    'r_hip_pitch' : 0.25479256152244334,
    'r_hip_roll' : 0.03672702367638624,
    'r_hip_yaw' : -0.005588860561231716,
    'r_knee' : -0.3000030972191293,
    'r_ankle_pitch' : -0.1489140270381754,
    'r_ankle_roll' : -0.03714938495742498,
}
state_machine_configurations['hands_final']['contacts'] = {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)}