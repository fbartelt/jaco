#%%
import time
import sys
import numpy as np
from importlib import import_module
from cvxopt import matrix, solvers
import plotly.express as px
sys.path.insert(1, '/home/fbartelt/Documents/UFMG/TCC/Sim/uaibot')
sys.path.insert(1, '/home/fbartelt/Coppelia/programming/zmqRemoteApi/clients/python/')
from zmqRemoteApi import RemoteAPIClient
from uaibot.robot import Robot
from uaibot.robot.links import Link
from uaibot.graphics.model3d import Model3D
from uaibot.utils import Utils
import pickle

solvers.options['show_progress'] = False

class CoppeliaInterface:
    """ simRemoteApi.start(19999)
    """
    def __init__(self, connectionAddress='127.0.0.1', connectionPort=19999, waitUntilConnected=True, doNotReconnectOnceDisconnected=True, timeOutInMs=5000, commThreadCycleInMs=5):
        self.connectionAddress = connectionAddress
        self.connectionPort = connectionPort
        self.waitUntilConnected = waitUntilConnected
        self.doNotReconnectOnceDisconnected = doNotReconnectOnceDisconnected
        self.timeOutInMs = timeOutInMs
        self.commThreadCycleInMs = commThreadCycleInMs
        self.client = RemoteAPIClient()
        self.zmqsim = self.client.getObject('sim')
        try:
            self.sim = import_module('sim')
        except:
            print ('--------------------------------------------------------------')
            print ('"sim.py" could not be imported. This means very probably that')
            print ('either "sim.py" or the remoteApi library could not be found.')
            print ('Make sure both are in the same folder as this file,')
            print ('or appropriately adjust the file "sim.py"')
            print ('--------------------------------------------------------------')
            print ('')
            self.sim = None
        self.clientID = self._init_sim()

    def _init_sim(self):
        if self.sim is None:
            return -1
        else:
            print ('Program started')
            self.sim.simxFinish(-1) # just in case, close all opened connections
            clientID = self.sim.simxStart(self.connectionAddress, self.connectionPort, self.waitUntilConnected, self.doNotReconnectOnceDisconnected, self.timeOutInMs, self.commThreadCycleInMs) # Connect to CoppeliaSim
            return clientID
    
    def test(self):
        if self.clientID!=-1:
            _ = self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)
            print ('Connected to remote API server')

            # Now try to retrieve data in a blocking fashion (i.e. a service call):
            res, objs = self.sim.simxGetObjects(self.clientID, self.sim.sim_handle_all, self.sim.simx_opmode_blocking)
            if res == self.sim.simx_return_ok:
                print ('Number of objects in the scene: ',len(objs))
            else:
                print ('Remote API function call returned with error code: ', res)

            time.sleep(2)

            # Now retrieve streaming data (i.e. in a non-blocking fashion):
            startTime = time.time()
            self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_mouse_x, self.sim.simx_opmode_streaming) # Initialize streaming
            while time.time() - startTime < 5:
                returnCode, data = self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_mouse_x, self.sim.simx_opmode_buffer) # Try to retrieve the streamed data
                if returnCode == self.sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
                    print ('Mouse position x: ', data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
                time.sleep(0.005)

            # Now send some data to CoppeliaSim in a non-blocking fashion:
            self.sim.simxAddStatusbarMessage(self.clientID, 'Hello CoppeliaSim!', self.sim.simx_opmode_oneshot)

            # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
            self.sim.simxGetPingTime(self.clientID)

            # Now close the connection to CoppeliaSim:
            self.sim.simxFinish(self.clientID)
        else:
            print ('Failed connecting to remote API server')
        print ('Program ended')
    
    def get_htms(self):
        res, joint_map = self.sim.simxGetObjects(self.clientID, self.sim.sim_object_joint_type, self.sim.simx_opmode_blocking)
        htms = []
        for i in joint_map:
            _, htm = self.sim.simxGetJointMatrix(self.clientID, i, self.sim.simx_opmode_blocking)
            aux = np.array([0,0,0,1]).reshape(1, 4)
            htm = np.block([[np.array(htm).reshape(3, 4)], [aux]])
            htms.append(htm)
        return htms
    
    def get_target_htm(self, target_path='/xd'):
        res, target_handle = self.sim.simxGetObjectHandle(self.clientID, target_path, self.sim.simx_opmode_blocking)
        _, target_position = self.sim.simxGetObjectPosition(self.clientID, target_handle, -1, self.sim.simx_opmode_blocking)
        _, target_orientation = self.sim.simxGetObjectOrientation(self.clientID, target_handle, -1, self.sim.simx_opmode_blocking)
        target_htm = np.round(np.array(self.zmqsim.buildMatrix(target_position, target_orientation)).reshape(3, 4), 6)
        aux = np.array([0,0,0,1]).reshape(1, 4)
        target_htm = np.block([[target_htm], [aux]])
        return target_htm
    
    def get_config(self, joint_handles=None):
        if joint_handles is None:
            _, joint_handles = self.sim.simxGetObjects(self.clientID, self.sim.sim_object_joint_type, self.sim.simx_opmode_blocking)
        q = []
        for handle in joint_handles:
            _, position = self.sim.simxGetJointPosition(self.clientID, handle, self.sim.simx_opmode_blocking)
            q.append(position)
        q = np.array(q).reshape(-1, 1).astype(float)
        return q
    
    def create_uaibot_robot(self, link_info=None):
        if link_info is None:
            link_info = self._get_dh()
        n = link_info.shape[1]
        links = []
        dummy = []
        dummy.append(Model3D(url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DaVinci/4.obj'))
        # print(type(dummy))
        for i in range(n):
            links.append(Link(i, theta=link_info[0, i], d=link_info[1, i], alpha=link_info[2, i], a=link_info[3, i], joint_type=link_info[4, i], list_model_3d=dummy))
        
        rob = Robot(name='jaco', links=links, list_base_3d_obj=dummy)
        return rob
    

jaco_links_info = np.array([[0, 0, np.pi/2, -np.pi/2,0,-np.pi],
                              [0, -0.1188, 0, -0.0098, -0.252, -0.0806],
                              [0, 0, -0.41, 0, 0, 0],
                              [0, -np.pi/2, -np.pi, np.pi/2, 0.3056*np.pi, 0.3056*np.pi],
                              [0, 0, 0, 0, 0, 0]])
#%%
sim = CoppeliaInterface(connectionPort=19997)
jaco = sim.create_uaibot_robot(jaco_links_info)
dt = 0.05
imax = int(20/dt)
htm_des = sim.get_target_htm()
q = sim.get_config()
_, joint_handles = sim.sim.simxGetObjects(sim.clientID, sim.sim.sim_object_joint_type, sim.sim.simx_opmode_blocking)

_ = sim.sim.simxStartSimulation(sim.clientID, sim.sim.simx_opmode_oneshot)
time.sleep(0.1)
q_hist, qdot_hist, r_hist = [], [], []
with open('inputs.pickle', 'rb') as f:
    u = pickle.load(f)

for i in range(imax):
    q = sim.get_config()
    r, jac_r = jaco.task_function(np.matrix(htm_des), q=q.astype(float))
    qdot = -(np.linalg.pinv(jac_r) * r)
    qdot = u
    
    for idx, handle in enumerate(joint_handles):
        sim.sim.simxSetJointTargetVelocity(sim.clientID, handle, qdot[idx][0], sim.sim.simx_opmode_oneshot)

    _ = sim.sim.simxSynchronousTrigger(sim.clientID)
    q_hist.append(q)
    qdot_hist.append(qdot)
    r_hist.append(r)

for idx, handle in enumerate(joint_handles):
        sim.sim.simxSetJointTargetVelocity(sim.clientID, handle, 0, sim.sim.simx_opmode_oneshot)


_ = sim.sim.simxStopSimulation(sim.clientID, sim.sim.simx_opmode_oneshot)

fig=px.line(np.array(q_hist).reshape(-1, 6))
fig.show()
fig=px.line(np.array(qdot_hist).reshape(-1, 6))
fig.show()
fig=px.line(np.array(r_hist).reshape(-1, 6))
fig.show()

# %%
sim = CoppeliaInterface(connectionPort=19997)
# [-0.14740046858787537, 0.1925376057624817, 0.8912556171417236]
dt = 0.05
imax = int(5/dt)
htm_des = sim.get_target_htm()
q = sim.get_config()
_, joint_handles = sim.sim.simxGetObjects(sim.clientID, sim.sim.sim_object_joint_type, sim.sim.simx_opmode_blocking)

_ = sim.sim.simxStartSimulation(sim.clientID, sim.sim.simx_opmode_oneshot)
time.sleep(0.1)
q_hist, qdot_hist, r_hist = [], [], []

for i in range(imax):
    q = sim.get_config()
    qdot = np.array([.1, .02, .03, .04, .05, .06])
    r = np.array([.1, .02, .03, .04, .05, .06])
    
    for idx, handle in enumerate(joint_handles):
        sim.sim.simxSetJointTargetVelocity(sim.clientID, handle, qdot[idx], sim.sim.simx_opmode_oneshot)

    _ = sim.sim.simxSynchronousTrigger(sim.clientID)
    q_hist.append(q)
    qdot_hist.append(qdot)
    r_hist.append(r)

for idx, handle in enumerate(joint_handles):
        sim.sim.simxSetJointTargetVelocity(sim.clientID, handle, 0, sim.sim.simx_opmode_oneshot)

for i in joint_handles:
    print(sim.sim.simxGetObjectPosition(sim.clientID, i, -1, sim.sim.simx_opmode_blocking))

input()
_ = sim.sim.simxStopSimulation(sim.clientID, sim.sim.simx_opmode_oneshot)

fig=px.line(np.array(q_hist).reshape(-1, 6))
fig.show()
fig=px.line(np.array(qdot_hist).reshape(-1, 6))
fig.show()
fig=px.line(np.array(r_hist).reshape(-1, 6))
fig.show()

# %%
