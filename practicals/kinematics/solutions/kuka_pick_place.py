#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.

INSTRUCTIONS:
    JUST RUN THE CODE. The RG2 gripper does collide with the robot wrist and the robot is stopped in simulation.

@Authors: Arturo Gil
@Time: April 2021

@Review: July 2023
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation


def pick(robot, gripper):

    target_positions = [[0.6, -0.1, 0.25],  # initial in front of conveyor
                        [0.6, 0.27, 0.25],  # pick the piece
                        [0.6, 0.0, 0.4]]  # drop the piece on the table
    target_orientations = [[-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2]]
    close_gripper = [False, True, True]
    # initial arm position
    q0 = np.array([0, 0, 0, -np.pi/2, 0, np.pi/4, 0])
    robot.secondary_objective = False

    robot.moveAbsJ(q_target=q0, precision=True)

    for i in range(len(target_positions)):
        q = robot.inversekinematics(target_position=target_positions[i],
                                    target_orientation=Euler(target_orientations[i]), q0=q0)
        robot.moveAbsJ(q_target=q, precision=True, endpoint=True)
        q0 = q
        if close_gripper[i]:
            gripper.close(precision=True)
        else:
            gripper.open(precision=True)


def place(robot, gripper, step_number):
    target_positions = [[0.2, -0.7, 0.4],  # over the table
                        [0.2, -0.7, 0.35]]  # drop the piece on the table
    target_orientations = [[-np.pi, 0, 0],
                           [-np.pi, 0, 0]]
    # change target points for drop zone
    target_positions[0] = target_positions[0] + step_number * np.array([0, 0.06, 0])
    target_positions[1] = target_positions[1] + step_number * np.array([0, 0.06, 0])

    close_gripper = [True, False, False]

    # initial arm position
    q0 = np.array([-np.pi/2, 0, 0, -np.pi / 2, 0, np.pi / 4, 0])
    robot.secondary_objective = False

    robot.moveAbsJ(q_target=q0, precision=False)

    for i in range(len(target_positions)):
        q = robot.inversekinematics(target_position=target_positions[i],
                                    target_orientation=Euler(target_orientations[i]), q0=q0)
        robot.moveAbsJ(q_target=q, precision=False)
        q0 = q
        if close_gripper[i]:
            gripper.close(precision=True)
        else:
            gripper.open(precision=True)


def pick_place():
    simulation = Simulation()
    simulation.start()
    robot = RobotKUKALBR(simulation=simulation)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start(name='/LBR_iiwa_14_R820/RG2/RG2_openCloseJoint')
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    for i in range(0, 6):
        pick(robot, gripper)
        place(robot, gripper, i)

    robot.plot_trajectories()
    simulation.stop()


if __name__ == "__main__":
    pick_place()
