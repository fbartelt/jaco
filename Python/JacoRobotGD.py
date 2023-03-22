# coding=utf-8
"""
(C) Copyright 2015 DQ MACRO Developers

This file is part of Poppeye

    DQ_jaco is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ_jaco is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with Little_John.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omaña  (juanjqogm@gmail.com)
Version History:

-- Initial Version
"""

#from DQ import *
#from DQ_kinematics import *
from dqrobotics import*
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH

import sys, os
sys.path.append(os.path.dirname(sys.path[0]))
import numpy as np
from math import pi
from math import sin, cos


from robot_dynamics.GaussDynamics import GaussDynamics
from dqrobotics.robot_modeling import DQ_SerialManipulator


def JacoRobotGD():

    #jaco_DH_theta = np.array([0,pi/2,pi/2,0,0,0])
    #jaco_DH_d = np.array([-0.1188,0,0,-0.252,-0.079,-0.04])
    #jaco_DH_a = np.array([0,-0.41,0,0,0,0])
    #jaco_DH_alpha = np.array([-pi/2,-pi,-pi/2,-0.3056*pi, 0.3056*pi , pi])

    jaco_DH_theta = np.array([0,pi/2,-pi/2,0,-pi, 0])  #pi/2
    jaco_DH_d = np.array([-0.1188,0,-0.0098,-0.252, -0.0806, 0])
    jaco_DH_a = np.array([0,-0.41,0,0,0,0])
    jaco_DH_alpha = np.array([-pi/2, -pi, pi/2, 0.3056*pi, 0.3056*pi, 0])
    jaco_DH_type = np.array([1,1,1,1,1,1])
    #This model is working, but it requires a recompute of the inertial parameters


    jaco_DH_matrix = np.array([jaco_DH_theta, jaco_DH_d, jaco_DH_a, jaco_DH_alpha, jaco_DH_type]) #jaco_dummy

    I = np.zeros((7, 3, 3))

    I[0, :, :] = np.array([[2.00000000e-02, -1.04749986e-09, -2.44450000e-09],
                            [-5.82000000e-10,  2.00000000e-02, -8.56591090e-17],
                            [-2.44450000e-09, -9.31500156e-10,  2.00000000e-02]])

    I[1, :, :] = np.array([[2.00000000e-02,  2.21828307e-18, -4.08486407e-18],
                          [2.21829066e-18,  2.00000000e-02, -1.25014681e-18],
                          [-4.08486407e-18, -1.25014678e-18,  2.00000000e-02]])

    I[2, :, :] = np.array([[2.00000000e-02, -1.05800084e-08,  1.04749986e-09],
                         [-9.65000844e-09,  2.00000000e-02,  9.31412730e-10],
                         [1.10599986e-09,  9.31404313e-10,  2.00000000e-02]])

    I[3, :, :] = np.array([[ 2.00000000e-02,  5.01695877e-11,  5.75555768e-09],
                         [-6.68165857e-11,  2.00000095e-02,  3.47743021e-09],
                         [5.58843443e-09,  3.46242837e-09, 1.99999905e-02]])

    I[4, :, :] = np.array([[1.00000000e-02,  4.68567586e-10, -5.36795610e-10],
                            [4.68802485e-10,  9.99999925e-03, -8.91455263e-10],
                            [-5.36881181e-10, -8.91455263e-10,  1.00000007e-02]])

    I[5, :, :] = np.array([[1.00000000e-02,  2.28602855e-23,  1.81903357e-23],
                         [2.38940138e-23,  1.00000000e-02,  3.23953619e-20],
                         [1.64380951e-23, -1.82757228e-19,  1.00000000e-02]])



    COM = np.array([[1.06300101e-05, -3.89500000e-02,  1.15199999e-04],
                    [2.05050000e-01, -3.16258847e-05,  2.32500000e-02],
                    [1.49498294e-03, -2.39000013e-03, -8.32499996e-02],
                    [-4.44913097e-04,  1.47478116e-02, -4.76383899e-03],
                    [-5.06594220e-04,  1.25010095e-02, -9.13026295e-03],
                    [5.84541042e-06, -2.05212812e-04,  7.36370554e-03]])

    mass = np.array([0.5, 0.5, 0.5, 0.5, 0.25, 0.25])

    jaco = GaussDynamics(jaco_DH_matrix, 'standard', I, mass, COM)
    #jaco = DQ_SerialManipulator(jaco_DH_matrix, 'standard')
    robot_base_offset = 1 + DQ.E * 0.5 * DQ([0, 0, 0, 0.15675])
    rotx180 = DQ([cos(pi / 2), sin(pi / 2), 0, 0])
    robot_base = robot_base_offset*rotx180
    jaco.set_base_frame(robot_base)
    jaco.set_reference_frame(robot_base)
    xeff = 1 + DQ.E*0.5*DQ([0,0,0,-0.052]) # without hand
    #xeff = 1 + DQ.E*0.5*DQ([0,0,0,-0.2])  #with hand
    jaco.set_effector(xeff)

    return jaco
