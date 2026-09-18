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
    q = [-3*np.pi/4, 0, 0, 0, np.pi/2, 0]
    robot.moveAbsJ(q_target=q, speed_factor=3.0)
    q = [3*np.pi/4, 0, 0, 0, np.pi/2, 0]
    robot.moveAbsJ(q_target=q, speed_factor=3.0)



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
    test_move_AbsJ(robot)
    simulation.stop()

