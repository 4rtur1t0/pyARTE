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
    # gripper = GripperRG2(simulation=simulation)
    # gripper.start()
    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    # Example with MoveAbsJ
    q1 = np.array([0, 0, 0, 0, 0, 0])
    q2 = np.array([-np.pi/2, -np.pi/3, -np.pi/4, np.pi/8, np.pi/8, 0])
    robot.moveAbsJ(q1, speed_factor=3)
    robot.reset_state()
    robot.moveAbsJ(q2, speed_factor=3)
    robot.plot_states()
    # reset
    robot.moveAbsJ(q1, speed_factor=3)
    robot.reset_state()

    # Example movements with moveJ
    tp1 = Vector([0.6, -0.5, 0.8])
    to1 = Euler([0, np.pi / 2, 0])
    tp2 = Vector([0.4, 0.4, 0.4])
    to2 = Euler([0, np.pi, 0])

    # mostrar en Coppelia los target points anteriores
    frame.show_target_points(target_positions=[tp1, tp2], target_orientations=[to1, to2])
    robot.moveJ(target_position=tp1, target_orientation=to1, speed_factor=5)
    robot.reset_state()
    robot.moveJ(target_position=tp2, target_orientation=to2, speed_factor=5)
    # robot.moveJ(target_position=tp1, target_orientation=to1, speed_factor=5)
    robot.plot_states()
    # robot.reset_state()

    # Example movements with moveL
    # robot.moveAbsJ(q1, speed_factor=3) # reset to an initial state
    robot.moveL(target_position=tp1, target_orientation=to1, speed_factor=5)
    robot.reset_state()
    robot.moveL(target_position=tp2, target_orientation=to2, speed_factor=5)
    # robot.moveL(target_position=tp1, target_orientation=to1, speed_factor=5)
    robot.plot_states()
    robot.reset_state()

    # Stop arm and simulation
    simulation.stop()
    # robot.plot_states()

