#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script computes the inverse kinematic of the IRB140 robot and sends joint values to Coppelia to view
the results.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


if __name__ == "__main__":
    simulation = Simulation()
    clientID = simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()
    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))


    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))
    # robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.105]), Euler([-np.pi / 2, 0, 0])))

    q0 = np.array([-np.pi/4, -np.pi/4, np.pi/8, -np.pi/4, np.pi / 4, -np.pi/4])

    ########################################################
    #
    # COMPLETE EL CÓDIGO
    # Defina las posiciones y orientaciones a alcanzar
    ########################################################
    target_positions = []
    target_orientations = []

    # mostrar en Coppelia los target points anteriores
    frame.show_target_points(target_positions, target_orientations)
    robot.moveAbsJ(q0)
    #for i in range(len(target_positions)):
        ####################################################
        # COMPLETE EL CÓDIGO
        # comande al robot a cada uno de los puntos deseados
        ####################################################


    # Stop arm and simulation
    simulation.stop()
    robot.plot_states()

