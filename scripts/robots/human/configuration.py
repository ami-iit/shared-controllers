import resolve_robotics_uri_py

# Model and ControlBoard
robot_name = "Human"
urdf_path = str(resolve_robotics_uri_py.resolve_robotics_uri("package://HDERviz/urdfs/humanSubject03_48dof.urdf"))
base_link = "Pelvis"

joints_list = ['jT9T8_rotx',
               'jT9T8_rotz',
               'jRightShoulder_rotx',
               'jRightShoulder_roty',
               'jRightShoulder_rotz',
               'jRightElbow_roty',
               'jRightElbow_rotz',
               'jLeftShoulder_rotx',
               'jLeftShoulder_roty',
               'jLeftShoulder_rotz',
               'jLeftElbow_roty',
               'jLeftElbow_rotz',
               'jLeftHip_rotx',
               'jLeftHip_roty',
               'jLeftHip_rotz',
               'jLeftKnee_roty',
               'jLeftKnee_rotz',
               'jLeftAnkle_rotx',
               'jLeftAnkle_roty',
               'jLeftAnkle_rotz',
               'jLeftBallFoot_roty',
               'jRightHip_rotx',
               'jRightHip_roty',
               'jRightHip_rotz',
               'jRightKnee_roty',
               'jRightKnee_rotz',
               'jRightAnkle_rotx',
               'jRightAnkle_roty',
               'jRightAnkle_rotz',
               'jRightBallFoot_roty',
               'jL5S1_roty']

remote_control_board_list =  ["/" + robot_name + "/HumanControlBoard"]

# Controller Configuration

## Control Mode
joints_control_mode = {}