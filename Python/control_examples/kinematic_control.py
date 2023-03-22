'''
Contributors:
- Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)

1) Open the CoppeliaSim scene jaco/scenes/kinematic_control.ttt
2) Run this script
3) Enjoy!
'''

from dqrobotics import *
from dqrobotics.interfaces.vrep  import DQ_VrepInterface
from dqrobotics.robot_control import ControlObjective
from dqrobotics.robot_control import DQ_PseudoinverseController
from robots.JacoRobot import JacoRobot
import time
from numpy import linalg as LA


vi = DQ_VrepInterface()

## Always use a try-catch in case the connection with V-REP is lost
## otherwise your clientid will be locked for future use
try:
    ## Connects to the localhost in port 19997 with timeout 100ms and 10 retries for each method call
    vi.connect(19997, 100, 10)
    vi.set_synchronous(True)

    ## Starts simulation in V-REP
    print("Starting V-REP simulation...")
    vi.start_simulation()
    time.sleep(0.1)

    jointnames = ("Jaco_joint1", "Jaco_joint2", "Jaco_joint3", "Jaco_joint4",
                   "Jaco_joint5", "Jaco_joint6")
    robot = JacoRobot()
    robot_base = vi.get_object_pose(jointnames[0])
    robot.set_reference_frame(robot_base)

    controller = DQ_PseudoinverseController(robot)
    controller.set_gain(0.5)
    controller.set_damping(0.05)
    controller.set_control_objective(ControlObjective.Translation)
    controller.set_stability_threshold(0.00001)

    xdesired = vi.get_object_pose("xd")
    i = 0
    #for i in range(iterations):
    while not controller.system_reached_stable_region():
        q = vi.get_joint_positions(jointnames)
        vi.set_object_pose("x", robot.fkm(q))
        u = controller.compute_setpoint_control_signal(q, vec4(xdesired.translation()))
        print("error: ", LA.norm(controller.get_last_error_signal()))
        print("Iteration: ", i)
        print("Is stable?: ", controller.system_reached_stable_region())
        vi.set_joint_target_velocities(jointnames, u)
        print("q_dot: ", vi.get_joint_velocities(jointnames))
        i=i+1
        vi.trigger_next_simulation_step()

    ## Stops simulation in V-REP
    print("Stopping V-REP simulation...")
    vi.stop_simulation()

    ## Disconnects V-REP
    vi.disconnect()

except Exception as exp:
    print(exp)
    print(
        "There was an error connecting to CoppeliaSim.")
    vi.disconnect_all()