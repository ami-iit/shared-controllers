import yarp

class FacialExpression(): 
    def __init__(self)->None:
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
        
    def update_face(self,ref):
        if(ref<0.8):
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