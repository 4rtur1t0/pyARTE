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


def test_move_AbsJ(robot):
    # test MoveAbsJ
    qs = [np.array([-np.pi/2, np.pi/4, np.pi/16, np.pi/2, -np.pi/2, 0]),
          np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
          np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
          np.array([np.pi/8, -np.pi/4, -np.pi/4, np.pi/8, np.pi/8, np.pi/8]),
          np.array([np.pi / 8, -np.pi / 4, -np.pi / 4, np.pi / 8, np.pi / 8, np.pi / 8]),
          np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
          np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
          np.array([-np.pi/2, np.pi/4, np.pi/16, 0, -np.pi/4, 0]),
          np.array([0, 0, 0, 0, np.pi / 2, 0])]
    # tests all different options of the moveAbsJ function
    for i in range(len(qs)):
        print('Targeting joint positions: ', i)
        robot.moveAbsJ(qs[i], speed_factor=5.0)
    for i in range(len(qs)):
        robot.moveAbsJ(qs[i], precision=False)
    for i in range(len(qs)):
        robot.moveAbsJ(qs[i], true_trapezoidal=False, precision=True)
    for i in range(len(qs)):
        robot.moveAbsJ(qs[i], true_trapezoidal=True, precision=True)
    for i in range(len(qs)):
        robot.moveAbsJ(qs[i], true_trapezoidal=True, precision=True,
                       speed_factor=3.0)


def test_moveJ(robot):
    positions = [Vector([0.5, -0.3, 0.7]),
                 Vector([0.5, 0.4, 0.6]),
                 Vector([0.5, -0.3, 0.2]),
                 Vector([-0.5, -0.3, 0.2]),
                 Vector([0.5, -0.3, 0.2]),
                 Vector([0.5, 0.0, 0.6]),
                 Vector([0.01, 0.01, 1.2])]
    orientations = [Euler([0, 0, 0]),
                    Euler([0, 0, 0]),
                    Euler([0, np.pi, 0]),
                    Euler([np.pi/8, np.pi, 0]),
                    Euler([np.pi/4, np.pi/2, np.pi/4]),
                    Euler([0, np.pi/2, 0]),
                    Euler([0, 0, 0])]
    for i in range(len(positions)):
        print('Targeting position and orientation: ', i)
        print(positions[i])
        print(orientations[i])
        robot.moveJ(target_position=positions[i],
                    target_orientation=orientations[i], speed_factor=3.0)



def test_moveJ_rangeq1(robot):
    robot.moveAbsJ(q_target=[-3*np.pi/4, 0, 0, 0, 0, 0], speed_factor=3.0)

    tp = Vector([-0.4, -0.4, 0.6])
    to = Euler([0, np.pi, 0])
    robot.moveJ(target_position=tp, target_orientation=to, speed_factor=3.0)

    tp = Vector([-0.4, 0.4, 0.6])
    to = Euler([0, np.pi, 0])
    robot.moveJ(target_position=tp, target_orientation=to, speed_factor=3.0)

def test_moveL_easy(robot):
    positions = [Vector([0.5, -0.5, 0.2]),
                 Vector([0.5, 0.5, 0.2]),
                 Vector([-0.5, 0.5, 0.3]),
                 Vector([0.5, 0.5, 0.3]),
                 Vector([0.5, 0.5, 0.7])]
    orientations = [Euler([0, np.pi, 0]),
                    Euler([0, np.pi, 0]),
                    Euler([0, np.pi, 0]),
                    Euler([0, np.pi/2, 0]),
                    Euler([0, 0, 0])]
    # Start with moveJ
    robot.moveJ(target_position=positions[0], target_orientation=orientations[0])
    for i in range(len(positions)):
        robot.moveL(target_position=positions[i], target_orientation=orientations[i],
                    precision=False,
                    plot=False,
                    speed_factor=3.0)


def test_moveL_difficult1(robot):
    positions = [Vector([-0.5, -0.5, 0.2]),
                 Vector([0.5, -0.5, 0.2]),
                 Vector([0.5, -0.5, 0.3]),
                 Vector([0.5, 0.5, 0.3]),
                 Vector([-0.5, 0.5, 0.4])]
    orientations = [Euler([0, np.pi, 0]),
                    Euler([0, np.pi, 0]),
                    Euler([0, np.pi, 0]),
                    Euler([0, np.pi, 0]),
                    Euler([0, np.pi, 0])]
    # Start with moveJ
    robot.moveJ(target_position=positions[0], target_orientation=orientations[0])
    for i in range(len(positions)):
        robot.moveL(target_position=positions[i], target_orientation=orientations[i], precision=False,
                    plot=False)


def test_moveL_difficult2(robot):
    positions = [Vector([-0.3, 0.3, 0.9]),
                 Vector([0.2, 0.2, 1.2])]
    orientations = [Euler([0, 0, 0]),
                    Euler([0, 0, 0])]
    # Start with moveJ
    robot.moveJ(target_position=positions[0], target_orientation=orientations[0])
    for i in range(len(positions)):
        robot.moveL(target_position=positions[i], target_orientation=orientations[i], precision=False,
                    plot=True)



def test_moveL_difficult3(robot):
    positions = [
                 Vector([-0.3, -0.3, 0.9]), # 0
                 Vector([0.3, 0.3, 1.1]),   # 1
                 Vector([-0.2, -0.2, 1.0]), # 2
                 Vector([0.3, -0.3, 1.0]),  # 3
                 Vector([0.3, 0.3, 0.9]),   # 4
                 Vector([-0.3, 0.3, 0.9]),  # 5
                 Vector([0.2, 0.2, 1.2])]   # 6
    orientations = [
                    Euler([0, 0, 0]),
                    Euler([0, 0, 0]),
                    Euler([np.pi/2, 0, 0]),
                    Euler([np.pi/2, 0, 0]),
                    Euler([0, 0, 0]),
                    Euler([0, 0, 0]),
                    Euler([0, 0, 0])]
    # Start with moveJ
    robot.moveJ(target_position=positions[0], target_orientation=orientations[0])
    q0 = robot.get_joint_positions()
    print(q0)
    for i in range(len(positions)):
        print('Targeting position/orientation: ', i)
        robot.moveL(target_position=positions[i], target_orientation=orientations[i],
                    precision=False,
                    plot=False,
                    extended=True,
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
    # gripper = GripperRG2(simulation=simulation)
    # gripper.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.05]), RotationMatrix(np.eye(3))))

    # test robot wait
    # robot.wait(20)
    # robot.wait_time(2)
    test_moveJ_rangeq1(robot)
    # test_move_AbsJ(robot)
    # test_moveJ(robot)
    # test_moveL_easy(robot)
    # test_moveL_difficult1(robot)
    # test_moveL_difficult2(robot)
    # test_moveL_difficult3(robot)
    simulation.stop()

