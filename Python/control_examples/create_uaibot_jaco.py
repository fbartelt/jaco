import sys
sys.path.insert(1, '/home/fbartelt/Documents/UFMG/TCC/Sim/uaibot/uaibot')
from robot.links import Link
import robot as rb
from utils import Utils
from graphics.meshmaterial import MeshMaterial
from graphics.model3d import Model3D
from simobjects.rigidobject import RigidObject
from simulation import Simulation
from simobjects.frame import Frame

import numpy as np

def _create_jaco(name='jaco_robot', color='#3e3f42', opacity=1):
    pi = np.pi
    # jaco_DH_theta = np.array([0,pi/2,-pi/2,0,-pi, 0])  #pi/2
    # jaco_DH_d = np.array([-0.1188,0,-0.0098,-0.252, -0.0806, 0])
    # jaco_DH_a = np.array([0,-0.41,0,0,0,0])
    # jaco_DH_alpha = np.array([-pi/2, -pi, pi/2, 0.3056*pi, 0.3056*pi, 0])
    
    jaco_DH_theta = np.array([0,         0,         0,      0,         0,         0])  #pi/2
    jaco_DH_d = np.array(    [0.1188,    0,         0.0098, 0.252,     0.0806,    0.052])
    jaco_DH_a = np.array(    [0,         0.41,      0,      0,         0     ,    0])
    jaco_DH_alpha = np.array([pi/2,      pi,       -pi/2,   0.3056*pi, 0.3056*pi, 0.3056*pi])

    jaco_DH_type = np.array([0, 0, 0, 0, 0, 0])
    link_info = np.array([jaco_DH_theta, jaco_DH_d, jaco_DH_alpha, jaco_DH_a, jaco_DH_type]) #jaco_dummy

    scale = 1
    n = link_info.shape[1]
    base_3d_obj = []
    mesh = MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5,
                        normal_scale=[0.5, 0.5], color=color,
                        opacity=opacity, side="DoubleSide")
    # original model is rotated (Robot fron = plane X x Y)

    Q00 = Utils.rotx(0)
    Q001 = Utils.trn([0, 0, 1.5675e-1])
    Q01 = Q001 * (Utils.rotz(link_info[0, 0]) * Utils.trn([0, 0, link_info[1, 0]]) * Utils.rotx(link_info[2, 0]) * Utils.trn(
        [link_info[3, 0], 0, 0]))
    Q02 = Q01 * (Utils.rotz(link_info[0, 1] + pi/2) * Utils.trn([0, 0, link_info[1, 1]]) * Utils.rotx(link_info[2, 1]) * Utils.trn(
        [link_info[3, 1], 0, 0]))
    Q03 = Q02 * (Utils.rotz(link_info[0, 2] - pi/2) * Utils.trn([0, 0, link_info[1, 2]]) * Utils.rotx(link_info[2, 2]) * Utils.trn(
        [link_info[3, 2], 0, 0]))
    Q04 = Q03 * (Utils.rotz(link_info[0, 3] + 0) * Utils.trn([0, 0, link_info[1, 3]]) * Utils.rotx(link_info[2, 3]) * Utils.trn(
        [link_info[3, 3], 0, 0]))
    Q05 = Q04 * (Utils.rotz(link_info[0, 4] - pi) * Utils.trn([0, 0, link_info[1, 4]]) * Utils.rotx(link_info[2, 4]) * Utils.trn(
        [link_info[3, 4], 0, 0]))
    #Q06 = Q05 * (Utils.rotz(link_info[0, 5] + 0) * Utils.trn([0, 0, link_info[1, 5]]) * Utils.rotx(link_info[2, 5]) * Utils.trn(
    #    [link_info[3, 5], 0, 0]))

    link0_mth = Utils.inv_htm(Q00)
    base_3d_obj = [
        Model3D(url='https://raw.githubusercontent.com/fbartelt/jaco/main/Python/robots/jacomodel/link1.obj',
                scale=scale, htm=link0_mth, mesh_material=mesh)]
    link_3d_obj = []
    link1_mth = Utils.inv_htm(Q01)
   
    link_3d_obj.append([
        Model3D(url='https://raw.githubusercontent.com/fbartelt/jaco/main/Python/robots/jacomodel/link2.obj',
                scale=scale, htm=link1_mth, mesh_material=mesh),
    ])

    link2_mth = Utils.inv_htm(Q02)
    link_3d_obj.append([
        Model3D(url='https://raw.githubusercontent.com/fbartelt/jaco/main/Python/robots/jacomodel/link3.obj',
                scale=scale, htm=link2_mth, mesh_material=mesh),
    ])

    link3_mth = Utils.inv_htm(Q03)
    link_3d_obj.append([
        Model3D(url='https://raw.githubusercontent.com/fbartelt/jaco/main/Python/robots/jacomodel/link4.obj',
                scale=scale, htm=link3_mth, mesh_material=mesh),
    ])

    link4_mth = Utils.inv_htm(Q04)
    link_3d_obj.append([
        Model3D(url='https://raw.githubusercontent.com/fbartelt/jaco/main/Python/robots/jacomodel/link5.obj',
                scale=scale, htm=link4_mth, mesh_material=mesh),
    ])

    link5_mth = Utils.inv_htm(Q05)
    link_3d_obj.append([
        Model3D(url='https://raw.githubusercontent.com/fbartelt/jaco/main/Python/robots/jacomodel/link6.obj',
                scale=scale, htm=link5_mth, mesh_material=mesh),
    ])

    link6_mth = Utils.trn([0,0,0])#Utils.inv_htm(Q06)
    # link_3d_obj.append([
    #     # conecta pá com bumerangue
    #     Model3D(url='https://raw.githubusercontent.com/fbartelt/jaco/main/Python/robots/jacomodel/link6.obj',
    #             scale=scale, htm=link6_mth, mesh_material=mesh),
    # ])

    links = []
    for i in range(n-1):
        links.append(Link(i, theta=link_info[0, i], d=link_info[1, i], alpha=link_info[2, i], a=link_info[3, i], joint_type=link_info[4, i],
                          list_model_3d=link_3d_obj[i]))

    q0 = [0, pi/2, -pi/2, 0, -pi]#, 0]
    htm_n_eef = Utils.rotz(-pi) * Utils.rotx(0.3056*pi) * Utils.rotx(0.3056*pi) * Utils.trn([0,0, 0.052])
    htm_base_0 = Utils.trn([0, 0, 1.5675e-1])#Utils.trn([-3.2712e-05, -1.7324e-05, 1.5675e-01])

    # Create joint limits
    joint_limits = np.matrix([[-3*np.pi, 3*np.pi], [-np.deg2rad(47), np.deg2rad(266)], 
                              [-np.deg2rad(19), np.deg2rad(322)], [-3*np.pi, 3*np.pi], 
                              [-3*np.pi, 3*np.pi]])#, [-np.pi, np.pi]])

    return links, base_3d_obj, htm_base_0, htm_n_eef, q0, joint_limits

def create_jaco(name='jaco_robot'):
    links, base_3d_obj, htm_base_0, htm_n_eef, q0, joint_limits = _create_jaco(name='jaco_robot')
    jaco = rb.Robot(name='jacojaco', links=links, list_base_3d_obj=base_3d_obj, htm=np.identity(4), 
                    htm_base_0=htm_base_0, htm_n_eef=htm_n_eef, q0=q0, eef_frame_visible=True, joint_limits=joint_limits)
    return jaco