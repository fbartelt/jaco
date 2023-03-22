cmake_minimum_required(VERSION 3.5)

project(kinematic_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
find_package (Threads REQUIRED)

add_library(JacoRobot JacoRobot.cpp)
target_link_libraries(JacoRobot
    Threads::Threads
    dqrobotics
)


add_executable(kinematic_control main.cpp)


target_link_libraries(kinematic_control
    pthread
    dqrobotics
    dqrobotics-interface-vrep
    JacoRobot
)

