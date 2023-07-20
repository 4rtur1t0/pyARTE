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
    q1 = np.array([np.pi/8, np.pi/8, np.pi/8, np.pi/8, np.pi/8, np.pi/8])
    q2 = np.array([-np.pi / 2, -np.pi/4, np.pi/8, np.pi/8, np.pi/8, np.pi/8])
    q3 = np.array([-np.pi / 2, 0, 0, -np.pi/8, -np.pi/8, np.pi/8])

    tp1 = Vector([0.6, -0.5, 0.8])
    to1 = Euler([0, np.pi / 2, 0])
    tp2 = Vector([0.6, 0.5, 0.5])
    to2 = Euler([0, np.pi / 2, 0])

    # COMMAND TO ABSOLUTE joint coordinates
    robot.moveAbsJ(q0, qdfactor=0.8, endpoint=True)
    robot.moveAbsJ(q1, qdfactor=0.8, endpoint=True)
    robot.moveAbsJ(q2, qdfactor=0.8, endpoint=True)
    robot.moveAbsJ(q3, qdfactor=0.8, endpoint=True)
    robot.moveAbsJ(q0, qdfactor=0.8, endpoint=True)

    # COMMAND TO specified target points
    robot.moveJ(target_position=tp1, target_orientation=to1, qdfactor=0.5, endpoint=True)
    robot.moveJ(target_position=tp2, target_orientation=to2, qdfactor=0.5, endpoint=True)
    robot.moveJ(target_position=tp1, target_orientation=to1, qdfactor=0.5, endpoint=True)

    # COMMANDING LINES in space
    frame.show_target_points(target_positions=[tp1, tp2], target_orientations=[to1, to2], wait_time=1)
    robot.moveL(target_position=tp2, target_orientation=to2, endpoint=True)
    robot.moveL(target_position=tp1, target_orientation=to1, endpoint=True)

    # BACK TO THE INITIAL POINT
    robot.moveAbsJ(q0, endpoint=True)

    # Stop arm and simulation
    simulation.stop()
    robot.plot_trajectories()

