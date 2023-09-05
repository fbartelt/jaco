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
- Juan Jose Quiroz Omaña  (juanjqo@g.ecc.u-tokyo.ac.jp)
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



def JacoRobot():
    jaco_DH_theta = np.array([0,pi/2,-pi/2,0,-pi, 0])  #pi/2
    jaco_DH_d = np.array([-0.1188,0,-0.0098,-0.252, -0.0806, 0])
    jaco_DH_a = np.array([0,-0.41,0,0,0,0])
    jaco_DH_alpha = np.array([-pi/2, -pi, pi/2, 0.3056*pi, 0.3056*pi, 0])
    jaco_DH_type = np.array([0,0,0,0,0,0])
    jaco_DH_matrix = np.array([jaco_DH_theta, jaco_DH_d, jaco_DH_a, jaco_DH_alpha, jaco_DH_type]) #jaco_dummy
    jaco = DQ_SerialManipulatorDH(jaco_DH_matrix)
    robot_base_offset = 1 + DQ.E * 0.5 * DQ([0, 0, 0, 0.15675])
    rotx180 = DQ([cos(pi / 2), sin(pi / 2), 0, 0])
    robot_base = robot_base_offset*rotx180
    jaco.set_base_frame(robot_base)
    jaco.set_reference_frame(robot_base)
    xeff = 1 + DQ.E*0.5*DQ([0,0,0,-0.052]) # without hand
    #xeff = 1 + DQ.E*0.5*DQ([0,0,0,-0.2])  #with hand
    jaco.set_effector(xeff)
    return jaco
