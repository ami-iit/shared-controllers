import os

# Model
box_urdf = os.environ.get('SMALL_BOX_MODEL')
if box_urdf:
    robot_name = "box"
    urdf_path = box_urdf
    base_link = "base_link"
else:
    print("Could not find the box model. Please set the environment variable SMALL_BOX_MODEL to the path of the box model.")


joints_list = ['joint1',
               'joint2']

remote_control_board_list =  ["/" + robot_name + "/BoxControlBoard"]

## Control Mode
joints_control_mode = {}
## Gains
controller_gains = {}
controller_gains['momentum_task'] = {}


controller_gains['momentum_task']['Ki'] = [50, 50, 200, 0, 0, 0]
controller_gains['momentum_task']['Kp'] = [1.5, 1.5, 2, 0.025, 0.025, 0.025]
