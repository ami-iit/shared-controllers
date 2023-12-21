import numpy as np
from qpsolvers import solve_qp

class PosturalTaskGain:
    def __init__(self, ndof):
        self.Kp = np.identity(ndof)
        self.Kd = np.identity(ndof)
        self.ndof = ndof

class MomentumControllerGain:
    def __init__(self):
        self.Kp = np.identity(6)
        self.Ki = np.identity(6)
        
class PosturalTaskController():

    def __init__(self, ndof):
        self.ndof = ndof
        self.gain = PosturalTaskGain(self.ndof)

        self.s_des = np.zeros(ndof)
        self.ds_des = np.zeros(ndof)
        

    def set_gain(self, gain : PosturalTaskGain):
        self.gain = gain

    def set_desired_posture(self, s, ds):
        self.s_des = s
        self.ds_des = ds

    def get_postural_task_torque(self, s, ds, M, Jc, h):

        Mb = M[0:6,0:6]
        Mbs = M[0:6,6:]
        Ms = M[6:,6:]
        Mb_inv = np.linalg.inv(Mb)
        
        Mbar = Ms-np.transpose(Mbs) @ Mb_inv @ Mbs

        s_err = s - self.s_des
        ds_err = ds - self.ds_des

        u_0  = - self.gain.Kp @ s_err - self.gain.Kd @ ds_err

        tau_0_model     = h[6:] - np.transpose(Mbs) @ Mb_inv @ h[0:6] + Mbar @ u_0 
        tau_0_sigma     = - np.transpose(Jc[:,6:]) + np.transpose(Mbs) @ Mb_inv @ np.transpose(Jc[:,0:6])

        return [tau_0_model, tau_0_sigma]

class MomentumController():

    def __init__(self, mass):
        self.gain = MomentumControllerGain()
        self.mass = mass

        self.p_com_des = np.zeros(3)
        self.dp_com_des = np.zeros(3)
        self.ddp_com_des = np.zeros(3)

        self.H_angular_des = np.zeros(3)

    def set_gain(self, gain : MomentumControllerGain):
        self.gain = gain

    def set_desired_center_of_mass_trajectory(self, p_com, dp_com, ddp_com):
        self.p_com_des = p_com
        self.dp_com_des = dp_com
        self.ddp_com_des = ddp_com

    def set_desired_angular_momentum(self, H_angular):
        self.H_angular_des = H_angular

    @staticmethod
    def get_rigid_contact_contraint(static_friction_coefficient, torsional_friction_coefficient, contact_area_size, fz_min):
        
        # CoP in support poligon
        A1 = np.array([[0, 0,  contact_area_size[0,0],  0,  1, 0],
                       [0, 0, -contact_area_size[0,1],  0, -1, 0],
                       [0, 0,  contact_area_size[1,0], -1,  0, 0],
                       [0, 0, -contact_area_size[1,1],  1,  1, 0]])
        b1 = np.zeros(4)

        # Torsional Friction
        A2 = np.array([[0, 0, -torsional_friction_coefficient, 0, 0,  1],
                       [0, 0, -torsional_friction_coefficient, 0, 0, -1]])
        b2 = np.zeros(2)

        # Unilateral Contact
        A3 = np.array([[0, 0, -1, 0, 0,  0]])
        b3 = np.array([-fz_min])

        # Friction Cone
        A4 = np.array([[ 1,  0, -static_friction_coefficient,  0,  0, 0],
                       [-1,  0, -static_friction_coefficient,  0,  0, 0],
                       [ 0,  1, -static_friction_coefficient,  0,  0, 0],
                       [ 0, -1, -static_friction_coefficient,  0,  0, 0]])
        b4 = np.zeros(4)


        Adeq = np.block([[A1], [A2], [A3], [A4] ]) 
        bdeq = np.concatenate((b1, b2, b3, b4)) 
        
        return [Adeq, bdeq]
    
    @staticmethod
    def tranform_local_wrench_task_into_global(A_local, b_local, w_H):
        w_R_i = w_H[:3,:3]
        i_i_X_i_w = np.block([[np.transpose(w_R_i), np.zeros([3, 3])],
                                  [np.zeros([3, 3]),  np.transpose(w_R_i)]])
        
        A_global = A_local @ i_i_X_i_w 
        b_global = b_local

        return [A_global, b_global]



    def get_momentum_control_tasks(self, H, p_com, w_H_frames):
        n_rows, _ = w_H_frames.shape
        n_contacts = np.floor_divide(n_rows, 4)
        
        Aeq = np.zeros([6, 6*n_contacts])
        beq = np.zeros(6)

        for idx_contact in range(n_contacts):

            # Contact Centroidal Jacobian
            w_H_i = w_H_frames[idx_contact*4:idx_contact*4+4, :]
            r_i   = w_H_i[0:3,3] - p_com
            skew_r_i = np.array([[0,      -r_i[2],  r_i[1]], 
                                 [r_i[2],       0, -r_i[0]], 
                                 [-r_i[1], r_i[0],       0]])
            G_X_i = np.block([[np.eye(3), np.zeros([3, 3])],
                              [skew_r_i,  np.eye(3)]]) 
            
            Aeq[:6, idx_contact*6:idx_contact*6+6] = G_X_i

        ddp_com_star = self.ddp_com_des - np.diag(self.gain.Ki[:3]) @ (p_com - self.p_com_des) -  np.diag(self.gain.Kp[:3]) @  (H[:3] / self.mass - self.dp_com_des)
        dH_angular_star = - np.diag(self.gain.Kp[:3]) @ H[3:]

        dH_star = np.concatenate((ddp_com_star * self.mass, dH_angular_star))
        f_grav  = np.array([0, 0, -9.81 * self.mass, 0, 0, 0])

        beq = dH_star - f_grav

        return [Aeq, beq]

def get_torques_projected_dynamics(tau_0_model, tau_0_sigma, Jc, Jf, Jdot_nu, M, h, B):

    M_inv = np.linalg.inv(M)
    J_M_inv = Jc @ M_inv

    Lambda = J_M_inv @ B
    
    pinv_Lambda = np.linalg.pinv(Lambda)[0]
    NullLambda  =  np.eye(Lambda.shape[1]) - pinv_Lambda @ Lambda

    tau_sigma = -pinv_Lambda @ J_M_inv @ np.transpose(Jf) + NullLambda @ tau_0_sigma
    tau_model = pinv_Lambda @ (J_M_inv @ h - Jdot_nu) + NullLambda @ tau_0_model

    return [tau_sigma, tau_model]

class WrenchQP():

    def __init__(self):
        pass

    def solve(self, tau_model, tau_sigma, Aeq, beq, Adeq, bdeq, reg, qpsolver="quadprog"):

        H = tau_sigma.T @ tau_sigma + reg * np.eye(tau_sigma.shape[1])
        c = tau_model @ tau_sigma

        return solve_qp(H, c, Adeq, bdeq, Aeq, beq,  solver=qpsolver)
    
    