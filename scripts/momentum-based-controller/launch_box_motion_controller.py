import time
import numpy as np
import signal
import matplotlib.pyplot as plt
import sys
import os
import yarp
import idyntree.bindings as iDynTree

from wholebodycontrollib import robotInterface
from wholebodycontrollib import wholebodycontrol
from wholebodycontrollib import wholebodylib
from wholebodycontrollib import profilerlib
from wholebodycontrollib import visualizer
from wholebodycontrollib import statemachine
from wholebodycontrollib import loggerplotterlib

from utils import configuration_handler


sys.path.append('..')
import robots.ergoCubGazeboV1.configuration as robot_configuration
import robots.small_box.configuration as box_configuration

# Flags
use_visualizer = False
use_logger = True
use_profiler = False
plot_data = True
save_figure = True

# Hands wrenches
consider_hands_wrenches = True
load = 1 # Kg mass of the top box
squeeze_force = 0 # N

if consider_hands_wrenches:
    load = 1
    f_l_hand_des = np.array([0,  squeeze_force, -9.81 * load /2 , 0, 0, 0])
    f_r_hand_des = np.array([0, -squeeze_force, -9.81 * load /2, 0, 0, 0])

# Frequency
controller_frequency = 0.004 # seconds



# Joints selector
idx_torque_controlled_joints = []
idx_positionDirect_controlled_joints = []

for joint_idx, joint in enumerate(robot_configuration.joints_list):
    if robot_configuration.joints_control_mode[joint] == 'torque':
        idx_torque_controlled_joints.append(joint_idx)
    elif robot_configuration.joints_control_mode[joint] == 'positionDirect':
        idx_positionDirect_controlled_joints.append(joint_idx)

idx_torque_controlled_joints_shifted = [idx + 6 for idx in idx_torque_controlled_joints]
idx_positionDirect_controlled_joints_shifted = [idx + 6 for idx in idx_positionDirect_controlled_joints]

# open kinDyn object
model = wholebodylib.robot(robot_configuration.urdf_path, robot_configuration.joints_list, robot_configuration.base_link)
box_model = wholebodylib.robot(box_configuration.urdf_path, [], box_configuration.base_link)

# open robotinterface
robot_interface = robotInterface.robotInterface(robot_configuration.robot_name, "/local", robot_configuration.joints_list, robot_configuration.remote_control_board_list)
robot_interface.open()


def termination():
    robot_interface.set_position_control_mode()
    robot_interface.robotDriver.close()

    fig_timer = None
    if use_profiler:
        profiler.print_timers_total_time()
        fig_timer = profiler.plot_timers(show_plot=False)

    fig_com = None
    fig_com_vel = None
    fig_ang_mom = None
    if use_logger:

        logger.save_data_to_file("data/logged_data_" + time.strftime("%Y%m%d-%H%M%S") + ".pickle")

        fig_com = logger.plot_data('t', ['p_com', 'p_com_des'], 'CoM position tracking', show_plot=False)
        fig_com_vel = logger.plot_data('t', ['v_com', 'v_com_des'], 'CoM velocity tracking', show_plot=False)
        fig_ang_mom = logger.plot_data('t', ['ang_mom', 'ang_mom_des'], 'Angular Momentum tracking', show_plot=False)

        fig_com_box = logger.plot_data('t', ['p_box_com', 'p_box_com_des'], 'Box CoM position tracking', show_plot=False)
        fig_ang_mom_box = logger.plot_data('t', ['ang_mom_box', 'ang_mom_box_des'], 'Box angular Momentum tracking', show_plot=False)


    if save_figure:
        # Create directory to save figure
        os.makedirs('figure', exist_ok=True)

        if fig_timer:
            fig_timer.savefig('figure/timers.png')
        if fig_com:
            fig_com.savefig('figure/com_position_tracking.png')
        if fig_com_vel:
            fig_com_vel.savefig('figure/com_velocity_tracking.png')
        if fig_ang_mom:
            fig_ang_mom.savefig('figure/angular_momentum_tracking.png')
        if fig_com_box:
            fig_com_box.savefig('figure/box_com_position_tracking.png')
        if fig_ang_mom_box:
            fig_ang_mom_box.savefig('figure/box_angular_momentum_tracking.png')

    if plot_data:
        plt.show()


# setup signal hanlder
def handler(signum, frame):
    print("Ctrl-c was pressed. Closing the application")
    termination()

    exit(1)
signal.signal(signal.SIGINT, handler)

# initialize controller gains
postural_task_controller = wholebodycontrol.PosturalTaskController(len(idx_torque_controlled_joints))
postural_task_gain = wholebodycontrol.PosturalTaskGain(len(idx_torque_controlled_joints))
Kp = np.array([robot_configuration.controller_gains['postural_task']['Kp'][joint_name] for joint_name in robot_configuration.joints_list])
postural_task_gain.Kp = np.diag(Kp[idx_torque_controlled_joints])
Kd = np.array([robot_configuration.controller_gains['postural_task']['Kd'][joint_name] for joint_name in robot_configuration.joints_list])
postural_task_gain.Kd = np.diag(Kd[idx_torque_controlled_joints])
postural_task_controller.set_gain(postural_task_gain)

momentum_controller = wholebodycontrol.MomentumController(model.kindyn.model().getTotalMass())
momentum_controller_gain = wholebodycontrol.MomentumControllerGain()
momentum_controller_gain.Ki = np.array(robot_configuration.controller_gains['momentum_task']['Ki'])
momentum_controller_gain.Kp = np.array(robot_configuration.controller_gains['momentum_task']['Kp'])
momentum_controller.set_gain(momentum_controller_gain)
momentum_controller.set_desired_center_of_mass_trajectory(np.zeros(3), np.zeros(3), np.zeros(3))

# Box
momentum_controller_box = wholebodycontrol.MomentumController(box_model.kindyn.model().getTotalMass())
momentum_controller_box_gain = wholebodycontrol.MomentumControllerGain()
momentum_controller_box_gain.Ki = np.array(box_configuration.controller_gains['momentum_task']['Ki'])
momentum_controller_box_gain.Kp = np.array(box_configuration.controller_gains['momentum_task']['Kp'])
momentum_controller_box.set_gain(momentum_controller_box_gain)
momentum_controller_box.set_desired_center_of_mass_trajectory(np.zeros(3), np.zeros(3), np.zeros(3))
momentum_controller_box.set_desired_angular_momentum(np.zeros(3))


# initialize QP
wrench_qp = wholebodycontrol.WrenchQP()
[Adeq_local, bdeq_local] = wholebodycontrol.MomentumController.get_rigid_contact_contraint(1/3, 1/75, np.array([[-0.12, 0.12], [-0.05, 0.05]]), 10)

# initialize state machine
state_machine = statemachine.StateMachine(repeat=False)

# initialize configurations
configurations = configuration_handler.statemachine_configurations_generator(robot_configuration, model, ["box_manipulation_1", "box_manipulation_2"], [10 ,10])

# Create selector matrix for the controlled joints
B_ctrl =  np.block([[np.zeros([6, len(idx_torque_controlled_joints)])], [np.eye(len(idx_torque_controlled_joints))]])


# initialize the visualizer
if use_visualizer:
    visualizer = visualizer.Visualizer()
    visualizer.add_model("Robot", model, robot_configuration.urdf_path, robot_configuration.base_link)
    visualizer.add_model("Desired", model, robot_configuration.urdf_path, robot_configuration.base_link, [0.8, 0.0, 0.0, 0.1])

    visualizer.run()

if use_logger:

    logger = loggerplotterlib.LoggerPlotter()
    logger.add_data_variables(['t', 'p_com', 'p_com_des', 'v_com', 'v_com_des', 'ang_mom', 'ang_mom_des', 'p_box_com', 'p_box_com_des', 'ang_mom_box', 'ang_mom_box_des' , 'joint_pos', 'joint_pos_des', 'tau'])

# # open port to read box pose
box_basestate_port = yarp.BufferedPortBottle()
box_basestate_port.open("/box/base/state:i")

yarp.Network.connect("/box/base/state:o",  box_basestate_port.getName())

box_pose_bottle = box_basestate_port.read(True)

time.sleep(0.01)

# Extracting the first three elements into one vector
box_pose_vector = [box_pose_bottle.get(i).asFloat64() for i in range(6)]
box_velocity = [box_pose_bottle.get(i).asFloat64() for i in range(6, 12)]

box_pose_idyn = iDynTree.Transform()
box_pose_idyn.setPosition(np.array(box_pose_vector[:3]))
box_pose_idyn.setRotation(iDynTree.Rotation().RPY(box_pose_vector[3], box_pose_vector[4], box_pose_vector[5]))

box_pose = box_pose_idyn.asHomogeneousTransform().toNumPy()

box_model.set_state(box_pose, [], box_velocity, [])

p_box_com_des = box_model.get_center_of_mass_position()



# reset clock
time_prev = time.time()
t = 0

if use_profiler:
    profiler = profilerlib.Profiler(time.time())
    profiler.add_timers(['Loop', 'Controller', 'DataReading', 'ComputeKinematics', 'StateMachine', 'LoopControl', 'QP', 'SetOutput'])

    if use_visualizer:
        profiler.add_timer(timer_name='Visualizer')
    if use_logger:
        profiler.add_timer(timer_name='Plotter')

first_run = True

# prepare robot
robot_interface.set_torque_control_mode(idx_torque_controlled_joints)
robot_interface.set_position_direct_control_mode(idx_positionDirect_controlled_joints)

while True:
    if use_profiler : profiler.start_timer(timer_name='Loop', now=time.time())
    if use_profiler : profiler.start_timer(timer_name='LoopControl', now=time.time())

    # # get box pose
    box_pose_bottle = box_basestate_port.read(False)

    if box_pose_bottle is not None:
        # Extracting the first 6 elements into one vector
        box_pose_vector = [box_pose_bottle.get(i).asFloat64() for i in range(6)]
        box_velocity_vector = [box_pose_bottle.get(i).asFloat64() for i in range(6, 12)]

        box_pose_idyn = iDynTree.Transform()
        box_pose_idyn.setPosition(np.array(box_pose_vector[:3]))
        box_pose_idyn.setRotation(iDynTree.Rotation().RPY(box_pose_vector[3], box_pose_vector[4], box_pose_vector[5]))

        box_pose = box_pose_idyn.asHomogeneousTransform().toNumPy()

        p_box_com_des  = box_pose_idyn.getPosition().toNumPy()
        r_box_com_des     = box_pose_idyn.getRotation().asRPY().toNumPy()

        pd_box_com_des = np.array(box_velocity_vector[:3])
        omega_box_com_des = np.array(box_velocity_vector[3:])

    # make p_box_com_des a circle on the y-z plane
    radius = 0.1
    p_box_com_des[1] = radius * np.cos(time.time()) + p_box_com_des[1]
    p_box_com_des[2] = radius * np.sin(time.time()) + p_box_com_des[2]

    pd_box_com_des[1] = -radius * np.sin(time.time())
    pd_box_com_des[2] =  radius * np.cos(time.time())

    # get kinematic state
    if use_profiler : profiler.start_timer(timer_name='DataReading', now=time.time())

    s = robot_interface.get_joints_position()
    ds = robot_interface.get_joints_velocity()
    base_pose = model.get_base_pose_from_contacts(s, {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)})
    w_b = model.get_base_velocity_from_contacts(base_pose, s, ds, ["l_sole", "r_sole"])


    if use_profiler : profiler.stop_timer(timer_name='DataReading', now=time.time())


    # get kinematic and dynamic quantities
    if use_profiler : profiler.start_timer(timer_name='ComputeKinematics', now=time.time())

    model.set_state(base_pose, s, w_b, ds)

    J_feet = model.get_frames_jacobian(["l_sole", "r_sole"])
    if consider_hands_wrenches:
        J_l_hand = model.get_frames_jacobian(["l_hand_palm"])
        J_r_hand = model.get_frames_jacobian(["r_hand_palm"])

    Jdot_nu_feet = model.get_frames_bias_acceleration(["l_sole", "r_sole"])
    if consider_hands_wrenches:
        Jdot_nu_l_hand = model.get_frames_bias_acceleration(["l_hand_palm"])
        Jdot_nu_r_hand = model.get_frames_bias_acceleration(["r_hand_palm"])

    if consider_hands_wrenches:
        w_H_frames = model.get_frames_transform(["l_sole", "r_sole", "l_hand_palm", "r_hand_palm"])
    else:
        w_H_frames = model.get_frames_transform(["l_sole", "r_sole"])


    box_model.set_state(box_pose, [], box_velocity, [])
    w_H_frames_box = box_model.get_frames_transform(["side1_contact_link", "side2_contact_link"])
    H_box = box_model.get_centroidal_momentum()
    p_box_com = box_model.get_center_of_mass_position()
    H_box_des = H_box

    M = model.get_mass_matrix()
    h = model.get_generalized_bias_force()
    Jcm = model.get_centroidal_momentum_jacobian()
    H = model.get_centroidal_momentum()
    p_com = model.get_center_of_mass_position()
    v_com = model.get_center_of_mass_velocity()


    if consider_hands_wrenches:
        Jf = np.vstack((J_feet,J_l_hand,J_r_hand))
        Jc = J_feet
        Jdot_nu = Jdot_nu_feet
    else:
        Jf = J_feet
        Jc = J_feet
        Jdot_nu = Jdot_nu_feet

    if use_profiler : profiler.stop_timer(timer_name='ComputeKinematics', now=time.time())


    # get desired configuration from state machine
    if use_profiler : profiler.start_timer(timer_name='StateMachine', now=time.time())
    if first_run:
        joint_pos_des = np.copy(s)
        p_com_des = np.copy(p_com)
        configuration_0 = statemachine.Configuration(joint_pos_des, p_com_des, 10.0)
        state_machine.add_configuration(configuration_0)
        for configuration in configurations:
            state_machine.add_configuration(configuration)

        state_machine.add_configuration(configuration_0)

        first_run = False


    if not state_machine.update(time.time()):
        break
    joint_pos_des, joint_vel_des, joint_acc_des, _, _, _ = state_machine.get_state()

    # Get center of mass trajectory from forward kinematics
    base_pose_des = model.get_base_pose_from_contacts(joint_pos_des, {'l_sole' : np.eye(4), 'r_sole' : np.eye(4)})
    w_b_des = model.get_base_velocity_from_contacts(base_pose_des, joint_pos_des, joint_vel_des, ["l_sole", "r_sole"])
    w_dot_b_des = model.get_base_acceleration_from_contacts(base_pose_des, w_b_des, joint_pos_des, joint_vel_des, joint_acc_des, ["l_sole", "r_sole"])

    model.set_state(base_pose_des, joint_pos_des, w_b_des, joint_vel_des)

    H_des = model.get_centroidal_momentum()
    v_com_des = model.get_center_of_mass_velocity()
    p_com_des = model.get_center_of_mass_position()
    acc_com_des = model.get_center_of_mass_acceleration(w_dot_b_des, joint_acc_des)
    postural_task_controller.set_desired_posture(joint_pos_des[idx_torque_controlled_joints], joint_vel_des[idx_torque_controlled_joints])

    momentum_controller.set_desired_center_of_mass_trajectory(p_com_des, v_com_des, acc_com_des)
    momentum_controller.set_desired_angular_momentum(H_des[3:])
    momentum_controller_box.set_desired_center_of_mass_trajectory(p_box_com_des, pd_box_com_des, np.zeros(3))
    momentum_controller_box.set_desired_angular_momentum(H_box_des[3:])

    if use_profiler : profiler.stop_timer(timer_name='StateMachine', now=time.time())

    # if no joints are controlled in torque, skip this part
    if idx_torque_controlled_joints:
        # compute postural and momentum controller
        if use_profiler : profiler.start_timer(timer_name='Controller', now=time.time())
        # select the controlled joints
        s_ctrl = s[idx_torque_controlled_joints]
        ds_ctrl = ds[idx_torque_controlled_joints]
        M_ctrl = np.block([[M[:6,:6], M[:6, idx_torque_controlled_joints_shifted]], [M[idx_torque_controlled_joints_shifted, :6], M[idx_torque_controlled_joints_shifted][:,idx_torque_controlled_joints_shifted]]])
        Jc_ctrl = np.block([Jc[:, :6], Jc[:, idx_torque_controlled_joints_shifted]])
        Jf_ctrl = np.block([Jf[:, :6], Jf[:, idx_torque_controlled_joints_shifted]])
        h_ctrl = np.concatenate((h[:6], h[idx_torque_controlled_joints_shifted]), axis=0)

        [tau_0_model, tau_0_sigma] = postural_task_controller.get_postural_task_torque(s_ctrl, ds_ctrl, M_ctrl, Jf_ctrl, h_ctrl)

        # Feet wrenches inequality constraints
        [Adeq_local_left_foot, _] = momentum_controller.tranform_local_wrench_task_into_global(Adeq_local, bdeq_local, w_H_frames[:4,:])
        [Adeq_local_right_foot, _] = momentum_controller.tranform_local_wrench_task_into_global(Adeq_local, bdeq_local, w_H_frames[4:8,:])
        Adeq = np.block([[Adeq_local_left_foot                         , np.zeros([Adeq_local_left_foot.shape[0], 6])],
                         [np.zeros([Adeq_local_right_foot.shape[0], 6]), Adeq_local_right_foot]])
        bdeq = np.block([bdeq_local, bdeq_local])

        [Aeq, beq] = momentum_controller.get_momentum_control_tasks(H, p_com, w_H_frames)
        [Aeq_box, beq_box] = momentum_controller_box.get_momentum_control_tasks(H_box, p_box_com, w_H_frames_box)
        # hands wrench task
        if consider_hands_wrenches:
            Aeq_hand = np.eye(6)
            beq_hand = np.zeros(6)
            Aeq_aug = np.block([[Aeq],
                               [np.zeros([6, 12]), Aeq_box[:, 6:], np.zeros([6, 6])],
                               [np.zeros([6, 12]), np.zeros([6, 6]), Aeq_box[:, :6]]])

            beq_aug = np.concatenate((beq, beq_box, beq_box))

            Adeq_aug = np.block([Adeq,  np.zeros([Adeq.shape[0], 12])])

        [tau_sigma, tau_model] = wholebodycontrol.get_torques_projected_dynamics(tau_0_model, tau_0_sigma, Jc_ctrl, Jf_ctrl, Jdot_nu, M_ctrl, h_ctrl, B_ctrl)


        if use_profiler : profiler.stop_timer(timer_name='Controller', now=time.time())

        # solve QP optimization
        if use_profiler : profiler.start_timer(timer_name='QP', now=time.time())
        if consider_hands_wrenches:
            f = wrench_qp.solve(tau_model, tau_sigma, Aeq_aug, beq_aug, Adeq_aug, bdeq, 0.01, "quadprog")
        else:
            f = wrench_qp.solve(tau_model, tau_sigma, Aeq, beq, Adeq, bdeq, 0.001, "quadprog")

        if f is None:
            print("Failed to solve the optimization, using previous value")
            break

        tau = tau_sigma @ f + tau_model


        if use_profiler : profiler.stop_timer(timer_name='QP', now=time.time())

    # set output torque
    if use_profiler : profiler.start_timer(timer_name='SetOutput', now=time.time())

    if idx_torque_controlled_joints:
        robot_interface.set_joints_torque(tau, np.array(idx_torque_controlled_joints))
    else:
        tau = []
    robot_interface.set_joints_position(joint_pos_des[idx_positionDirect_controlled_joints], np.array(idx_positionDirect_controlled_joints))

    if use_profiler : profiler.stop_timer(timer_name='SetOutput', now=time.time())

    # Update the visualier
    if use_visualizer:
        if use_profiler : profiler.start_timer(timer_name='Visualizer', now=time.time())

        visualizer.update_model("Robot", base_pose, s)

        visualizer.update_model("Desired", base_pose, joint_pos_des)

        if use_profiler : profiler.stop_timer(timer_name='Visualizer', now=time.time())

    if use_logger:
        if use_profiler : profiler.start_timer(timer_name='Plotter', now=time.time())

        logger_data = { 't' : t,
                        'p_com'     : p_com, 'p_com_des'     :     p_com_des,
                        'v_com'     : v_com, 'v_com_des'     :     v_com_des,
                        'p_box_com' : p_box_com, 'p_box_com_des' : p_box_com_des,
                        'ang_mom'   : H[3:], 'ang_mom_des'   :     H_des[3:],
                        'ang_mom_box'   : H_box[3:], 'ang_mom_box_des'   :      H_box_des[3:],
                        'joint_pos' :     s, 'joint_pos_des' : joint_pos_des,
                        'tau' : tau}
        logger.append_data(logger_data)

        if use_profiler : profiler.stop_timer(timer_name='Plotter', now=time.time())

    if use_profiler:
        profiler.stop_timer("LoopControl", time.time())

    # wait and update timer
    dt = time.time() - time_prev
    if dt < controller_frequency:
        time.sleep(controller_frequency - dt)
    else:
        print("Deadline miss: " + str(dt) + " [s]")

    if use_profiler:
        profiler.stop_timer("Loop", time.time())

    dt = time.time() - time_prev
    t = t+dt
    time_prev = time.time()


termination()

