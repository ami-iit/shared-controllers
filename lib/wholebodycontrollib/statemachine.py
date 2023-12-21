import numpy as np
import yarp

class Configuration():

    def __init__(self, joint_position : np.array, com_position : np.array, duration):
        self.joint_position = joint_position
        self.com_position = com_position
        self.duration = duration

class StateMachine():

    def __init__(self, repeat = False):
        self.configurations = []
        self.time = 0
        self.state_start_time = -1
        self.current_state = 0
        self.repeat = repeat
        self.phi = 0.0

        # rpc for face expression
        yarp.Network.init()

        rpc_client_name = '/ergoCubEmotions/rpcClient'
        rpc_server_name = '/ergoCubEmotions/rpc'

        self.rpc_command = yarp.Bottle()
        self.rpc_command.addString('setEmotion')
        self.rpc_response = yarp.Bottle()

        self.rpc_client = yarp.RpcClient()
        self.rpc_client.open(rpc_client_name)

        yarp.Network.connect(rpc_client_name, rpc_server_name)

        # Initialize face expression to angry
        self.happy = False
        self.rpc_command.addString('shy')
        self.rpc_client.write(self.rpc_command, self.rpc_response)
        self.rpc_command.pop()
        
    
    def add_configuration(self, configuration : Configuration):
        self.configurations.append(configuration)
    
    def compute_offset(self, ref):
         return 0.5*(ref-0.85)
     
    def update(self, time) -> bool:
        self.time = time

        if self.current_state == 0 or (time - self.state_start_time >= self.configurations[self.current_state].duration):
            self.state_start_time = time
            self.current_state += 1
            print("changing to state: " + str(self.current_state))

            if self.current_state >= len(self.configurations):
                if self.repeat:
                    self.current_state = 0
                else:
                    self.current_state = len(self.configurations) - 1
                    return False
        
        return True

    def get_state(self, use_parametrized=False, ref=0, pos=0, J=0, period=0.001, tracking_gain=0.1):

        time_since_start = self.time - self.state_start_time

        tau = time_since_start / self.configurations[self.current_state].duration

        if tau > 1.0:
            tau = 1
            
        joint_position_initial = self.configurations[self.current_state - 1].joint_position
        joint_position_final = self.configurations[self.current_state].joint_position

        com_position_initial = self.configurations[self.current_state - 1].com_position
        com_position_final = self.configurations[self.current_state].com_position

        if self.current_state==1 and use_parametrized:
            self.pos_offset = pos - ref

        if self.current_state==2 and use_parametrized:

            # ref = ref + (self.pos_offset)
            ref = ref + self.compute_offset(ref)
            # print('ref ' + str(ref))
            # print('pos ',str(pos))

            if ref < 0.8:
                if ref > pos:
                    tracking_gain  = tracking_gain
                if ref < pos and ref>0.7:
                    tracking_gain =  tracking_gain * ((ref - 0.7)/0.1)
                if ref<0.7:
                    tracking_gain = 0

                if self.happy:
                    self.rpc_command.addString('shy')
                    self.rpc_client.write(self.rpc_command, self.rpc_response)
                    self.rpc_command.pop()
                    self.happy = False
            else:
                if not self.happy:
                    self.rpc_command.addString('happy')
                    self.rpc_client.write(self.rpc_command, self.rpc_response)
                    self.rpc_command.pop()
                    self.happy = True


            phi_dot =  -tracking_gain * (pos - ref )

            delta_phi = period * phi_dot
            self.phi = self.phi + delta_phi

            if self.phi > 1.0:
                self.phi = 1.0
            elif self.phi <0.0:
                self.phi = 0.0

            joint_position = joint_position_initial + (joint_position_final - joint_position_initial) * self.phi
            joint_velocity = phi_dot * (joint_position_final - joint_position_initial)
            joint_acceleration = 0 * (joint_position_final - joint_position_initial)

            com_position = com_position_initial + (com_position_final - com_position_initial) * self.phi
            com_velocity = phi_dot * (com_position_final - com_position_initial)
            com_acceleration = 0 * (com_position_final - com_position_initial)
        else:

            joint_position = joint_position_initial + (joint_position_final - joint_position_initial) * (10.0 * (tau)**3 - 15.0 * (tau)**4 + 6.0 * (tau)**5)
            joint_velocity = (joint_position_final - joint_position_initial) * (30.0 * (tau)**2 - 60.0 * (tau)**3 + 30.0 * (tau)**4)
            joint_acceleration = (joint_position_final - joint_position_initial) * (60.0 * (tau) - 180.0 * (tau)**2 + 120.0 * (tau)**3)

            com_position = com_position_initial + (com_position_final - com_position_initial) * (10.0 * (tau)**3 - 15.0 * (tau)**4 + 6.0 * (tau)**5)
            com_velocity = (com_position_final - com_position_initial) * (30.0 * (tau)**2 - 60.0 * (tau)**3 + 30.0 * (tau)**4)
            com_acceleration = (com_position_final - com_position_initial) * (60.0 * (tau) - 180.0 * (tau)**2 + 120.0 * (tau)**3)

        return joint_position, joint_velocity, joint_acceleration, com_position, com_velocity, com_acceleration



         


