#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


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

    q1 = np.array([-np.pi/4, np.pi/8, -np.pi/8, np.pi/4, -np.pi/4, np.pi/4])
    q2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q3 = np.array([np.pi/8, 0, -np.pi/4, 0, -np.pi/4, 0])
    q4 = np.array([0, 0, 0, 0, np.pi / 2, 0])

    gripper.open()
    gripper.close()
    gripper.open()
    T = robot.directkinematics(q1)
    robot.moveAbsJ(q1, precision=False)
    robot.moveAbsJ(q2, precision=True)
    robot.moveAbsJ(q3)
    robot.moveAbsJ(q4)

    robot.stop_joints()
    simulation.stop()

