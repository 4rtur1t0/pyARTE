#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script commands the robot to different target points.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from robots.abbirb140.abbirb140 import RobotABBIRB140
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

    qA = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    qB = np.array([np.pi/2, np.pi/6, -np.pi/6, np.pi/8, np.pi/9, np.pi/10])
    # qC = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # qD = np.array([-np.pi / 2, -np.pi / 6, np.pi / 6, -np.pi / 2, -np.pi / 2, -np.pi / 2])

    # COMMAND TO ABSOLUTE joint coordinates
    robot.moveAbsJ(qA, precision=True, traj_type='linear_approximation')
    robot.moveAbsJ(qB, precision=True, traj_type='linear_approximation')
    # robot.moveAbsJ(qC, precision=True, traj_type='linear_approximation')
    # robot.moveAbsJ(qD, precision=True, traj_type='linear_approximation')
    robot.moveAbsJ(qA, precision=True, traj_type='linear_approximation')

    # Stop arm and simulation
    simulation.stop()
    robot.plot_states()

