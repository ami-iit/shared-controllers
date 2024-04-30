import numpy as np
import idyntree.bindings as iDynTree
import copy 
from enum import Enum
from typing import Union
# from adam.casadi.computations import KinDynComputations

class TransformFormat(Enum):
    HOMOGENOUS = 1
    ADJOINT = 2
    ADJOINT_WRENCH = 3


class robot():

    def __init__(self, urdf_path, joints_list, base_link, kinDyn:Union [iDynTree.KinDynComputations, None]=None):

        if kinDyn is None:
            self.joints_list = joints_list
            self.base_link = base_link
            self.urdf_path = urdf_path
            self.ndof = len(joints_list)

            # Load Model and Initialize KinDyn
            self.kindyn = iDynTree.KinDynComputations()
            mdlLoader = iDynTree.ModelLoader()
            mdlLoader.loadReducedModelFromFile(urdf_path, joints_list)
            if not self.kindyn.loadRobotModel(mdlLoader.model()):
                raise ValueError('Failed to load the robot model in iDynTree.')
            self.initialize_buffer_variables()
        else: 
            self.kindyn = kindyn
            self.ndof = kindyn.getNrOfDegreesOfFreedom()
            self.initialize_buffer_variables()


    def initialize_buffer_variables(self): 
        self.s = iDynTree.VectorDynSize(self.ndof)
        self.ds = iDynTree.VectorDynSize(self.ndof)
        self.H_b = iDynTree.Transform()
        self.w_b = iDynTree.Twist()
        self.gravity = iDynTree.Vector3()
        self.gravity.zero()
        self.gravity.setVal(2, -9.81)

    def get_base_pose_from_contacts(self, s, contact_frames_pose : dict):
        self.s = iDynTree.VectorDynSize.FromPython(s)
        self.ds.resize(self.ndof)
        self.ds.zero()
        self.w_b = iDynTree.Twist()
        self.w_b.zero()

        self.kindyn.setRobotState(self.s, self.ds, self.gravity)

        w_p_b = np.zeros(3)
        w_H_b = np.eye(4)

        for key, value in contact_frames_pose.items():
            w_H_b_i = value @ self.kindyn.getRelativeTransform(key, self.kindyn.getFloatingBase()).asHomogeneousTransform().toNumPy()
            w_p_b = w_p_b + w_H_b_i[0:3, 3]
        
        w_H_b[0:3, 3] = w_p_b / len(contact_frames_pose)
        # for the time being for the orientation we are just using the orientation of the last contact
        w_H_b[0:3, 0:3] = w_H_b_i[0:3, 0:3]

        return w_H_b

    def get_base_velocity_from_contacts(self, H_b, s, ds, contact_frames_list : list):
        self.s = iDynTree.VectorDynSize.FromPython(s)
        self.ds = iDynTree.VectorDynSize.FromPython(ds)
        self.w_b.zero()
        self.H_b.fromHomogeneousTransform(iDynTree.Matrix4x4.FromPython(H_b))

        self.kindyn.setRobotState(self.H_b, self.s, self.w_b, self.ds, self.gravity)

        Jc_multi_contacts = self.get_frames_jacobian(contact_frames_list)
        w_b = np.linalg.lstsq(Jc_multi_contacts[:,0:6], -Jc_multi_contacts[:,6:] @ ds, rcond=-1)[0]

        return w_b
    
    def get_base_acceleration_from_contacts(self, H_b, w_b, s, ds, dds, contact_frames_list : list):
        self.s = iDynTree.VectorDynSize.FromPython(s)
        self.ds = iDynTree.VectorDynSize.FromPython(ds)
        self.w_b = iDynTree.Twist.FromPython(w_b)
        self.H_b.fromHomogeneousTransform(iDynTree.Matrix4x4.FromPython(H_b))

        self.kindyn.setRobotState(self.H_b, self.s, self.w_b, self.ds, self.gravity)

        Jc_multi_contacts = self.get_frames_jacobian(contact_frames_list)
        Jc_nu_multi_contacts = self.get_frames_bias_acceleration(contact_frames_list)

        w_dot_b = np.linalg.lstsq(Jc_multi_contacts[:,0:6], -Jc_multi_contacts[:,6:] @ dds - Jc_nu_multi_contacts, rcond=-1)[0]

        return w_dot_b

    def set_state(self, H_b, s, w_b, ds):
        self.s = iDynTree.VectorDynSize.FromPython(s)
        self.ds = iDynTree.VectorDynSize.FromPython(ds)
        self.w_b = iDynTree.Twist.FromPython(w_b)
        self.H_b.fromHomogeneousTransform(iDynTree.Matrix4x4.FromPython(H_b))

        self.kindyn.setRobotState(self.H_b, self.s, self.w_b, self.ds, self.gravity)

    def forward_kinematic(self, frame_name):
        w_H_f = self.kindyn.getWorldTransform(frame_name)
        return w_H_f.asHomogeneousTransform().toNumPy()
    
    def get_frame_transform(self, frame_name):
        w_H_f = self.kindyn.getWorldTransform(frame_name)
        return w_H_f.asHomogeneousTransform().toNumPy()

    def get_frames_transform(self, frames_list : list):
        w_H_frames = np.zeros([4*len(frames_list), 4])
        for idx, frame_name in enumerate(frames_list):
            w_H_f = self.kindyn.getWorldTransform(frame_name)
            w_H_frames[idx*4:(idx*4+4),:] = w_H_f.asHomogeneousTransform().toNumPy()

        return w_H_frames
    
    def get_frames_relative_trasform(self, reference_frame : str, frame : str, transform_format = TransformFormat.HOMOGENOUS):
        w_H_f = self.kindyn.getRelativeTransform(reference_frame, frame)
        iDynTree.Transform()
        if transform_format == TransformFormat.HOMOGENOUS:
            return w_H_f.asHomogeneousTransform().toNumPy()
        elif transform_format == TransformFormat.ADJOINT:
            return w_H_f.asAdjointTransform().toNumpy()
        elif transform_format == TransformFormat.ADJOINT_WRENCH:
            return w_H_f.asAdjointTransformWrench().toNumpy()

    def get_frames_jacobian(self, frames_list : list):
        Jc_frames = np.zeros([6*len(frames_list), 6+self.ndof])

        for idx, frame_name in enumerate(frames_list):
            Jc = iDynTree.MatrixDynSize(6,6+self.ndof)
            self.kindyn.getFrameFreeFloatingJacobian(frame_name, Jc)

            Jc_frames[idx*6:(idx*6+6),:] = Jc.toNumPy()
        
        return Jc_frames
    
    def get_center_of_mass_jacobian(self):
        Jc_com = iDynTree.MatrixDynSize(3,6+self.ndof)
        self.kindyn.getCenterOfMassJacobian(Jc_com)

        return Jc_com.toNumPy()

    def get_frames_bias_acceleration(self, frames_list : list):
        Jdot_nu_frames = np.zeros(6*len(frames_list))

        for idx, frame_name in enumerate(frames_list):
            Jdot_nu = self.kindyn.getFrameBiasAcc(frame_name)
            Jdot_nu_frames[idx*6:(idx*6+6)] = Jdot_nu.toNumPy()
        
        return Jdot_nu_frames
    
    def get_center_of_mass_bias_acceleration(self):
        Jdot_nu_com = self.kindyn.getCenterOfMassBiasAcc()

        return Jdot_nu_com.toNumPy()

    def get_mass_matrix(self):
        M = iDynTree.MatrixDynSize(6+self.ndof,6+self.ndof)
        self.kindyn.getFreeFloatingMassMatrix(M)

        return M.toNumPy()

    def get_generalized_bias_force(self):
        h = iDynTree.FreeFloatingGeneralizedTorques(self.kindyn.model())
        self.kindyn.generalizedBiasForces(h)

        return np.concatenate((h.baseWrench().toNumPy(), h.jointTorques().toNumPy()))

    def get_centroidal_momentum_jacobian(self):
        Jcm = iDynTree.MatrixDynSize(6,6+self.ndof)
        self.kindyn.getCentroidalTotalMomentumJacobian(Jcm)

        return Jcm.toNumPy()
    
    def get_centroidal_momentum(self):
        Jcm = self.get_centroidal_momentum_jacobian()
        nu = np.concatenate((self.w_b.toNumPy(), self.ds.toNumPy()))
        return Jcm @ nu

    def get_center_of_mass_position(self):
        p = iDynTree.Position()
        p = self.kindyn.getCenterOfMassPosition()

        return p.toNumPy()
    
    def get_center_of_mass_velocity(self):
        v = iDynTree.Vector3()
        v = self.kindyn.getCenterOfMassVelocity()

        return v.toNumPy()
    
    def get_center_of_mass_acceleration(self, w_dot_b, dds):

        J_com = self.get_center_of_mass_jacobian()
        J_nu_com = self.get_center_of_mass_bias_acceleration()

        a = J_com @ np.concatenate((w_dot_b, dds)) + J_nu_com

        return a

    def get_torques_selector_matrix(self):
        return np.block([[np.zeros([6, self.ndof])], [np.eye(self.ndof)]])

