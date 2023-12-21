# Model
robot_name = "box"
urdf_path = "/home/icub/software/dic-iit/element_ergonomy-control/build/install/share/ergonomy-control/robots/small_box/small_box.urdf"
base_link = "base_link"

## Gains
controller_gains = {}
controller_gains['momentum_task'] = {}

controller_gains['momentum_task']['Ki'] = [50, 50, 200, 0, 0, 0]
controller_gains['momentum_task']['Kp'] = [ 0,  0,   0, 0.025, 0.025, 0.025]
