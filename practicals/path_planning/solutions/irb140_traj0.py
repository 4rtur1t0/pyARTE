#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script commands the robot to different target points.

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


    # set the TCP of the RG2 gripper
    # robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.105]), Euler([-np.pi / 2, 0, 0])))

    tp1 = Vector([0.6, -0.5, 0.8])
    to1 = Euler([0, np.pi / 2, 0])
    tp2 = Vector([0.6, 0.3, 0.5])
    to2 = Euler([0, np.pi / 2, np.pi/2])

    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.moveAbsJ(q0, precision=True)

    T = robot.directkinematics(q0)

    # COMMAND TO specified target points
    robot.moveJ(target_position=tp1, target_orientation=to1)
    robot.moveJ(target_position=tp2, target_orientation=to2)
    robot.moveJ(target_position=tp1, target_orientation=to1)

    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q1 = np.array([np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0])
    q2 = np.array([-np.pi / 2, -np.pi/4, np.pi/8, np.pi/8, np.pi/8, np.pi/8])
    q3 = np.array([-np.pi / 2, 0, 0, -np.pi/8, -np.pi/8, np.pi/8])

    tp1 = Vector([0.6, -0.5, 0.8])
    to1 = Euler([0, np.pi / 2, 0])
    tp2 = Vector([0.6, 0.5, 0.5])
    to2 = Euler([0, np.pi / 2, 0])

    gripper.open()
    # COMMAND TO ABSOLUTE joint coordinates
    robot.moveAbsJ(q0, precision=True)
    robot.moveAbsJ(q1, precision=True)
    robot.moveAbsJ(q2, precision=True)
    robot.moveAbsJ(q3, precision=True)
    robot.moveAbsJ(q2, precision=True)
    robot.moveAbsJ(q1, precision=True)
    robot.moveAbsJ(q0, precision=True)
    gripper.close()

    # COMMAND TO specified target points
    robot.moveJ(target_position=tp1, target_orientation=to1)
    robot.moveJ(target_position=tp2, target_orientation=to2)
    robot.moveJ(target_position=tp1, target_orientation=to1)

    # COMMANDING LINES in space
    robot.moveL(target_position=tp2, target_orientation=to2)
    robot.moveL(target_position=tp1, target_orientation=to1)
    gripper.open()
    # BACK TO THE INITIAL POINT
    robot.moveAbsJ(q0)

    # Stop arm and simulation
    simulation.stop()
    robot.plot_states()

