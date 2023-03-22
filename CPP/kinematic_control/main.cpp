/*
  - Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)

    Prerequisites:
    - dqrobotics
    - dqrobotics-interface-vrep

    Instructions:
    1) Open the CoppeliaSim scene jaco/scenes/kinematic_control.ttt
    2) Run this script
    3) Enjoy!
*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>
#include "JacoRobot.h"


using namespace Eigen;

int main(void)
{
    DQ_VrepInterface vi;
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::vector<std::string>jointnames = {"Jaco_joint1", "Jaco_joint2", "Jaco_joint3", "Jaco_joint4",
                                          "Jaco_joint5", "Jaco_joint6"};

    // robot definition
    auto robot = std::make_shared<DQ_SerialManipulatorDH>
            (JacoRobot::kinematics());

    //Update the base of the robot from CoppeliaSim
    robot->set_reference_frame(vi.get_object_pose(jointnames[0]));


    DQ_PseudoinverseController controller(robot);
    controller.set_gain(0.5);
    controller.set_damping(0.05);
    controller.set_control_objective(DQ_robotics::Translation);
    controller.set_stability_threshold(0.00001);

    DQ xdesired = vi.get_object_pose("xd");

    int i=0;
    while (not controller.system_reached_stable_region())
    {
        VectorXd q = vi.get_joint_positions(jointnames);
        vi.set_object_pose("x", robot->fkm(q));
        VectorXd u = controller.compute_setpoint_control_signal(q, vec4(xdesired.translation()));
        std::cout << "task error: " <<controller.get_last_error_signal().norm()
                  <<" Iteration: "<<i<<std::endl;
        vi.set_joint_target_velocities(jointnames, u);
        vi.trigger_next_simulation_step();
        i++;

    }
    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}
