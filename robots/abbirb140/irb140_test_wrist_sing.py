#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/abbirb140.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def test_moveL_wrist1(robot):
    """
    Move linearly from one target to the next in a linear fashion
    """
    positions = [Vector([0.5, -0.5, 0.71]),
                 Vector([0.5, 0.5, 0.71])]
    orientations = [Euler([0, np.pi/2, 0]),
                    Euler([0, np.pi/2, 0])]
    for i in range(0, len(positions), 2):
        # Start with moveJ
        robot.moveJ(target_position=positions[i], target_orientation=orientations[i],
                    precision=True)
        robot.moveL(target_position=positions[i+1], target_orientation=orientations[i+1],
                    precision=False,
                    plot=True,
                    speed_factor=3.0,
                    debug=False)


def test_moveL_wrist2(robot):
    """
    Move specifically to target points with q5=0 (wrist singularity, with q4 not zero)
    """
    joint_positions = [[-np.pi/2, 0.0, -np.pi/16, np.pi/4, 0.0, -np.pi/4],
                       [-np.pi/8, np.pi/8, 0,  -np.pi/4, 0.0, -np.pi / 8]]
    for i in range(0, len(joint_positions), 2):
        robot.moveAbsJ(q_target=joint_positions[i])
        T = robot.directkinematics(joint_positions[i+1])
        robot.moveL(target_position=T.pos(), target_orientation=T.R(),
                    precision=True,
                    plot=True,
                    speed_factor=1.0,
                    debug=False)

def test_moveL_wrist3(robot):
    """
    Move specifically to target points with q5=0 (wrist singularity, with q4 not zero)
    """
    q = [-np.pi/3, 0, -np.pi/16, np.pi/8, 0.0, np.pi/3]
    T = robot.directkinematics(q)
    robot.moveAbsJ(q_target=q)
    robot.moveL(target_position=T.pos()+np.array([0, 0, 0.2]),
                target_orientation=T.R(),
                extended=False,
                plot=True)
    robot.moveL(target_position=T.pos()+np.array([0, 0, -0.2]),
                target_orientation=T.R(),
                extended=False,
                precision=True,
                plot=True,
                speed_factor=1.0)


def test_moveL_wrist4(robot):
    """
    Move specifically to target points with q5=0 (wrist singularity, with q4 not zero)
    """
    joint_positions = [[-np.pi/4, 0, 0, np.pi/8, np.pi/8, np.pi/8],
                       [-np.pi / 8, 0, 0, -np.pi / 4, 0, -np.pi / 8],
                       [np.pi/4, 0, 0, np.pi / 8, np.pi / 4, np.pi / 8],
                       [-np.pi / 4, np.pi / 16, 0, np.pi / 8, -np.pi / 2, np.pi / 8],
                       [-np.pi / 8, np.pi / 16, 0, -np.pi / 4, 0, -np.pi / 8],
                       [0, 0, 0, np.pi / 8, 0, np.pi / 8],
                       [np.pi / 4, 0, 0, 0, 0, -np.pi / 8]]
    # Start with moveJ
    robot.moveAbsJ(q_target=joint_positions[0])
    for i in range(len(joint_positions)):
        T = robot.directkinematics(joint_positions[i])
        robot.moveL(target_position=T.pos(), target_orientation=T.R(),
                    precision=True,
                    plot=True,
                    speed_factor=1.0)

def test_moveL_wrist2_try_mission(robot):
    """
    Move specifically to target points with q5=0 (wrist singularity, with q4 not zero)
    """
    q0 = [-np.pi/2, 0.0, -np.pi/16, np.pi/4, 0.0, -np.pi/4]
    qf = [-np.pi/8, np.pi/8, 0,  -np.pi/4, 0.0, -np.pi / 8]
    T0 = robot.directkinematics(q0)
    qinv0 = robot.inverse_kinematics(target_position=T0.pos(),
                                     target_orientation=T0.R(),
                                     extended=False,
                                     q0=q0)
    Tf = robot.directkinematics(qf)
    for i in range(qinv0.shape[1]):
        robot.moveAbsJ(q_target=qinv0[:, i], speed_factor=5.0, precision=True)
        robot.moveL(target_position=Tf.pos(), target_orientation=Tf.R(),
                    precision=True,
                    plot=True,
                    speed_factor=1.0,
                    debug=False)


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    # Connect to the robot
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    # test_moveL_wrist2(robot)
    test_moveL_wrist2_try_mission(robot)


    #test_moveL_wrist1(robot)
    #test_moveL_wrist3(robot)
    # test_moveL_wrist4(robot)
    simulation.stop()

