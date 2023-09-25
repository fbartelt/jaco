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
from uaibot.simulation import Simulation
from simobjects.frame import Frame
from create_uaibot_jaco import create_jaco
import pickle

solvers.options['show_progress'] = False

#%%
class CoppeliaInterface:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.client.setStepping(True)
    
    def _get_joint_handles(self):
        # Types: 10 - revolute, 11 - prismatic, 12 - spherical
        handles = self.sim.getObjectsInTree(self.sim.handle_scene, self.sim.object_joint_type)
        types = list(map(lambda x: self.sim.getJointType(x)!=10, handles))
        return handles, types

    def _get_htms(self):
        joint_handles, _ = self._get_joint_handles()
        aux = np.array([0,0,0,1]).reshape(1, 4)
        htms = []
        for idx, handle in enumerate(joint_handles[:-1]):
            htm = self.sim.getObjectMatrix(joint_handles[idx+1], handle)
            htm = np.block([[np.array(htm).reshape(3, 4)], [aux]])
            htms.append(htm)
        return np.round(htms, 6)
    
    def _get_htms2(self):
        joint_handles, _ = self._get_joint_handles()
        aux = np.array([0,0,0,1]).reshape(1, 4)
        htms = []
        htm0 = np.round(self.sim.poseToMatrix(self.sim.getObjectPose(joint_handles[0], self.sim.handle_world)), 6)
        htm0 = np.block([np.identity(3), np.array([0,0,0]).reshape(-1, 1)])
        htms.append(np.block([[np.array(htm0).reshape(3, 4)], [aux]]))

        for idx, handle in enumerate(joint_handles[:-1]):
            htm = np.round(self.sim.poseToMatrix(self.sim.getObjectPose(joint_handles[idx+1], handle)), 6)
            htm = np.block([[np.array(htm).reshape(3, 4)], [aux]])
            htms.append(htm)
        return htms
    
    def _get_dh(self):
        htms = self._get_htms2()
        _, types = self._get_joint_handles()
        links_info = np.zeros((5,1))

        for i, htm in enumerate(htms):
            # theta = np.arctan(htm[1, 0]/ htm[0, 0])
            # alfa = np.arctan(htm[2, 1]/ htm[2, 2])
            theta = np.arctan2(htm[1, 0], htm[0, 0])
            alfa = np.arctan2(htm[2, 1], htm[2, 2])
            a = htm[0, 3] / np.cos(theta)
            d = htm[2, 3]
            type_ = types[i]
            link_info = np.array([theta, d, alfa, a, type_]).reshape(-1, 1)
            # link_info = np.array([d, np.rad2deg(theta), a, np.rad2deg(alfa), type_]).reshape(-1, 1)
            links_info = np.block([links_info, link_info])
        links_info = links_info[:, 1:]

        links_info = np.array([[0, 0, np.pi/2, -np.pi/2,0,-np.pi],
                              [0, -0.1188, 0, -0.0098, -0.252, -0.0806],
                              [0, 0, -0.41, 0, 0, 0],
                              [0, -np.pi/2, -np.pi, np.pi/2, 0.3056*np.pi, 0.3056*np.pi],
                              [0, 0, 0, 0, 0, 0]]) #jaco
        # links_info = np.array([[0, 0     , np.deg2rad(150), np.pi  , np.deg2rad(-120), -np.pi , np.deg2rad(150)],
        #                       [0, 0.2    , 0              , 0.4    , 0               , 0.39   , 0],
        #                       [0, 0      , 0              , 0      , 0               , 0      , 0],
        #                       [0, np.pi/2, np.pi/2        , np.pi/2, np.pi/2         , np.pi/2, np.pi/2],
        #                       [0, 0, 0, 0, 0, 0, 0]]) # abb

        return links_info
    
    # def _get_dh2(self):
    #     handles, types = self._get_joint_handles()
    #     links_info = np.zeros((5,1))
    #     aux = np.array([0,0,0,1]).reshape(1, 4)
    #     htms = []

    #     for idx, handle in enumerate(handles[:-1]):
    #         htm = np.round(self.sim.poseToMatrix(self.sim.getObjectPose(handles[idx+1], handle)), 6)
    #         htm = np.block([[np.array(htm).reshape(3, 4)], [aux]])
    #         htms.append(htm)

    #     for i, htm in enumerate(htms):
    #         theta = np.arctan2(htm[1, 0], htm[0, 0])
    #         alfa = np.arctan2(htm[2, 1], htm[2, 2])
    #         a = htm[0, 3] / np.cos(theta)
    #         d = htm[2, 3]
    #         type_ = types[i]
    #         # link_info = np.array([theta, d, alfa, a, type_]).reshape(-1, 1)
    #         link_info = np.array([d, np.rad2deg(theta), a, np.rad2deg(alfa), type_]).reshape(-1, 1)
    #         links_info = np.block([links_info, link_info])
    #     links_info = links_info[:, 1:]

    #     links_info = np.array([[0, np.pi/2, -np.pi/2,0,-np.pi, 0],
    #                           [-0.1188, 0, -0.0098, -0.252, -0.0806, 0],
    #                           [0, -0.41, 0, 0, 0, 0],
    #                           [-np.pi/2, -np.pi, np.pi/2, 0.3056*np.pi, 0.3056*np.pi, 0],
    #                           [0, 0, 0, 0, 0, 0]])

    #     return links_info
    
    def OLD_create_uaibot_robot(self):
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
    
    def create_uaibot_robot(self):
        return create_jaco()
    
    def get_target_htm(self, target_path='/xd'):
        target_handle = self.sim.getObject(target_path)
        target_htm = np.round(self.sim.poseToMatrix(self.sim.getObjectPose(target_handle, self.sim.handle_world)), 6)
        aux = np.array([0,0,0,1]).reshape(1, 4)
        target_htm = np.block([[np.array(target_htm).reshape(3, 4)], [aux]])
        return target_htm
    
    def get_config(self, joint_handles=None):
        if joint_handles is None:
            joint_handles, _ = self._get_joint_handles()
        q = []
        for handle in joint_handles:
            q.append(self.sim.getJointPosition(handle))
        q = np.array(q).reshape(-1, 1).astype(float)
        return q
## Debug Juancho
translation_quaternion = [0.2, 0.2, 0.5]
rotation_quaternion = [0, -8.38e-6, 0, 1]

#%%
sim = CoppeliaInterface()
# print(sim._get_dh().T)
rob = sim.create_uaibot_robot()
dt = sim.sim.getSimulationTimeStep()

#%%
""" TESTE NOVO"""
time_max = 10
eps = 0.05
imax = round(time_max / dt)

joint_handles, _ = sim._get_joint_handles()
q = sim.get_config(joint_handles)[:-1]
q_map = np.array(-q.copy() - rob.q.copy())
q = -q - q_map

jac_eef, htm_eef = rob.jac_geo(axis='eef', q=q)
htm_des = np.matrix(sim.get_target_htm('/xx'))
# htm_des = htm_eef * Utils.trn([0, 0.5, -0.2])
# htm_des = np.array([[-0.15302 , -0.496994,  0.854156, -0.102123],
#        [-0.843845, -0.384117, -0.374672, -0.187982],
#        [ 0.514305, -0.778108, -0.360609, -0.095886],
#        [ 0.      ,  0.      ,  0.      ,  1.      ]])
_, htms = rob.jac_geo(axis='dh')

n = len(htms)
Kt = 2
xi = 1
q_hist, qdot_hist, r_hist = [], [], []
sim.sim.startSimulation()
print('Program started')
time.sleep(0.1)

i=0

def evaluate_error(r, tol_pos=5e-3, tol_ori=5):
    error_pos = max(abs(r[0:3]))[0, 0]
    if r.shape[0] == 6:
        error_ori = (180 / np.pi) * max(abs(np.arccos(1 - r[3:6])))[0, 0]
    else:
        error_ori = 0
    ok1 = error_pos < tol_pos
    ok2 = error_ori < tol_ori if len(r.tolist()) > 3 else True

    return bool(ok1 and ok2), error_pos, error_ori

converged = False

while (not converged) and (i < imax):
    q = -sim.get_config(joint_handles)[:-1] - q_map
    qdot_max = rob.joint_limit[:, 1] - q.reshape(-1, 1)
    qdot_min = -(rob.joint_limit[:, 0] - q.reshape(-1, 1))

    if i % 50 == 0 or i == imax - 1:
        sys.stdout.write('\r')
        sys.stdout.write("[%-20s] %d%%" % ('=' * round(20 * i / (imax - 1)), round(100 * i / (imax - 1))))
        sys.stdout.flush()

    r, jac_r = rob.task_function(np.matrix(htm_des), q=q.astype(float))
    # r = r[:3]
    # jac_r = jac_r[:3, :]
    
    H = 2 * (jac_r.T * jac_r) + eps * np.identity(n)
    f = 2*(Kt * r.T @ jac_r).T
    
    A = np.block([[np.identity(n)], [-np.identity(n)]])
    b = np.block([[xi * qdot_max ], [-xi * qdot_min]])

    try:
        qdot = solvers.qp(matrix(H), matrix(f))['x']#, matrix(A), matrix(b))['x']
    except:
        qdot = np.matrix(np.zeros((n, 1)))
        error_qp = True

    # qdot = -Kt*np.array(Utils.dp_inv(jac_r, 0.002) @ (r)).reshape(n, 1)
    qdot = np.array(qdot).reshape(n, 1)
    # qdot = np.array([0.3,0.3,0.3,0.3,0.3,]).reshape(n, 1)

    for idx, handle in enumerate(joint_handles[:-1]):
        # if i < 100:
        #     sim.sim.setJointTargetVelocity(handle, u[i%11][idx])
        # else:
        #     sim.sim.setJointTargetVelocity(handle, 0)
        sim.sim.setJointTargetVelocity(handle, -qdot[idx][0])
        # if idx in [0, 1, 2, 3, 4, 5]:
            # sim.sim.setJointTargetVelocity(handle, 0.3)
        # else:
        #     sim.sim.setJointTargetVelocity(handle, 0)
    
    q = (q + qdot*dt).astype(float)
    q_hist.append(q)
    qdot_hist.append(qdot)
    r_hist.append(r)
    rob.add_ani_frame(i * dt, q)
    # print(sim.sim.getSimulationTime())
    sim.client.step()
    i += 1
    converged, *_ = evaluate_error(r, tol_pos=1e-3, tol_ori=2)

sim.sim.pauseSimulation()

# [0.03306747227907181, 0.23839274048805237, 0.8409093618392944]
# [-0.7882683277130127, -0.12436030805110931, -0.6026344299316406, 0.033067476004362106, -0.42973530292510986, 0.8122178316116333, 0.3944994807243347, 0.2383929342031479, 0.4404103755950928, 0.5699445605278015, -0.6936875581741333, 0.8409093618392944]
# [0.03306744620203972, 0.23839303851127625, 0.8409093618392944, 0.15264460444450378, -0.9074929356575012, -0.2656891345977783, 0.2873423993587494]
#%%
sim.sim.stopSimulation()

ubsim = Simulation.create_sim_grid([rob])
frame_des = Frame(name='htm_des', htm=htm_des)
ubsim.add(frame_des)
ubsim.run()
#%%
"""












"""
#%%
time_max = 5
eps = 0.05
imax = round(time_max / dt)

joint_handles, _ = sim._get_joint_handles()
q = sim.get_config()

jac_eef, htm_eef = rob.jac_geo(axis='eef', q=q)
# htm_des = np.matrix(sim.get_target_htm('/xx'))
htm_des = np.round(Utils.rotx(np.pi/2) @ Utils.trn([0.5, 0, -0.2]) @ htm_eef, 6)
htm_des = np.round(Utils.trn([0, 0, -0.5]) @ htm_eef, 6)
sim.sim.setObjectPose(39, joint_handles[-1], sim.sim.matrixToPose(list(np.array(htm_des[:-1, :]).ravel())))

n = len(htms)
Kt = 0.5
xi = 1
q_hist, qdot_hist, r_hist = [], [], []
sim.sim.startSimulation()
print('Program started')
time.sleep(0.1)
with open('inputs.pickle', 'rb') as f:
    u = pickle.load(f)

for i in range(imax):
    qdot_max = np.ones(rob.q.shape)*10
    qdot_min = -np.ones(rob.q.shape)*10
    q = sim.get_config(joint_handles)

    if i % 50 == 0 or i == imax - 1:
        sys.stdout.write('\r')
        sys.stdout.write("[%-20s] %d%%" % ('=' * round(20 * i / (imax - 1)), round(100 * i / (imax - 1))))
        sys.stdout.flush()

    r, jac_r = rob.task_function(np.matrix(htm_des), q=q.astype(float))
    
    H = 2 * (jac_r.T * jac_r + eps * np.identity(n))
    f = 2*(Kt * r.T @ jac_r).T
    
    A = np.block([[np.identity(n)], [-np.identity(n)]])
    b = np.block([[xi * qdot_max], [-xi * qdot_min]])

    # qdot = np.array(Utils.dp_inv(jac_r, 0.002) @ (r + dhtm_des[:3, -1])).reshape(n, 1)
    try:
        qdot = solvers.qp(matrix(H), matrix(f), matrix(A), matrix(b))['x']
    except:
        qdot = np.matrix(np.zeros((n, 1)))
        error_qp = True

    # qdot = -(np.linalg.pinv(jac_r) * r)
    qdot = np.array(qdot).reshape(n, 1)

    for idx, handle in enumerate(joint_handles):
        # if i < 100:
        #     sim.sim.setJointTargetVelocity(handle, u[i%11][idx])
        # else:
        #     sim.sim.setJointTargetVelocity(handle, 0)
        sim.sim.setJointTargetVelocity(handle, qdot[idx][0])
        # if idx in [0, 1, 2, 3, 4, 5]:
            # sim.sim.setJointTargetVelocity(handle, 0.3)
        # else:
        #     sim.sim.setJointTargetVelocity(handle, 0)
    
    q = (q + qdot*dt).astype(float)
    q_hist.append(q)
    qdot_hist.append(qdot)
    r_hist.append(r)
    rob.add_ani_frame(i * dt, q)
    sim.client.step()

sim.sim.stopSimulation()

print('Program ended')
fig=px.line(np.array(q_hist).reshape(-1, 6))
fig.show()
fig=px.line(np.array(qdot_hist).reshape(-1, 6))
fig.show()
fig=px.line(np.array(r_hist).reshape(-1, 6))
fig.show()

#%%
""" old
"""

class CoppeliaWrapper:
    """ simRemoteApi.start(19999)
    """
    def __init__(self, connectionAddress='127.0.0.1', connectionPort=19999, waitUntilConnected=True, doNotReconnectOnceDisconnected=True, timeOutInMs=5000, commThreadCycleInMs=5):
        self.connectionAddress = connectionAddress
        self.connectionPort = connectionPort
        self.waitUntilConnected = waitUntilConnected
        self.doNotReconnectOnceDisconnected = doNotReconnectOnceDisconnected
        self.timeOutInMs = timeOutInMs
        self.commThreadCycleInMs = commThreadCycleInMs
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
    
# simulation = CoppeliaWrapper(connectionPort=19997)
# simulation.clientID
#%%

links = []
for i in range(n):
    links.append(Link(i, theta=link_info[0, i], d=link_info[1, i], alpha=link_info[2, i], a=link_info[3, i], joint_type=link_info[4, i],
                        list_model_3d=link_3d_obj[i]))
#%%

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,100, 5)

if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(2)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()
    sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
    while time.time()-startTime < 5:
        returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
        if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
        time.sleep(0.005)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

#%%
# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/ikMovementViaRemoteApi.ttt
#
# Do not launch simulation, then run this script

from zmqRemoteApi import RemoteAPIClient

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')
cop = CoppeliaInterface()

joint_handles, _ = cop._get_joint_handles()

tipHandle = sim.getObject('/LBR4p/tip')
targetHandle = sim.getObject('/LBR4p/target')

# Set-up some movement variables:
maxVel = 0.1
maxAccel = 0.01
maxJerk = 80

# Start simulation:
sim.startSimulation()

def cb(pose,vel,accel,handle):
    sim.setObjectPose(handle,-1,pose)


# Send movement sequences:
# initialPose = sim.getObjectPose(tipHandle,-1)
# targetPose = [0, 0, 0.85, 0, 0, 0, 1]
# sim.moveToPose(-1,initialPose,[maxVel],[maxAccel],[maxJerk],targetPose,cb,targetHandle,[1,1,1,0.1])

# targetPose = [
#     0, 0, 0.85,
#     -0.7071068883, -6.252754758e-08, -8.940695295e-08, -0.7071067691
# ]
# sim.moveToPose(-1,sim.getObjectPose(tipHandle,-1),[maxVel],[maxAccel],[maxJerk],targetPose,cb,targetHandle,[1,1,1,0.1])

# sim.moveToPose(-1,sim.getObjectPose(tipHandle,-1),[maxVel],[maxAccel],[maxJerk],initialPose,cb,targetHandle,[1,1,1,0.1])
for i in range(100):
    for handle in joint_handles:
        sim.setJointTargetVelocity(handle, 0.2)
    # client.step()
sim.stopSimulation()

print('Program ended')

# %%

sim = CoppeliaInterface()
rob = sim.create_uaibot_robot()