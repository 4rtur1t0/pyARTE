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
from robots.abbirb140 import RobotABBIRB140
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))

    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q1 = np.array([-np.pi/2, np.pi/8, np.pi/8, np.pi/8, np.pi/8, np.pi/8])

    # COMMAND TO ABSOLUTE joint coordinates
    robot.moveAbsJ(q0, qdfactor=.2, precision=True, endpoint=True)
    robot.moveAbsJ(q1, qdfactor=1.0, precision=False, endpoint=True)
    robot.moveAbsJ(q0, qdfactor=0.2, precision=True, endpoint=True)

    # Stop arm and simulation
    simulation.stop()
    robot.plot_trajectories()

