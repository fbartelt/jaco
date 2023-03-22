#pragma once
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>

namespace DQ_robotics
{

class JacoRobot
{
public:
    static DQ_SerialManipulatorDH kinematics();
};
}
