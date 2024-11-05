#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_project_2425.ttt scene before running this script.

The code beneath must be completed by the student in order to produce different assembly of the ARUCO Tetris pieces
according to their ids.

@Authors: Arturo Gil
@Time: October 2024
"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import ReferenceFrame
from robots.simulation import Simulation
from robots.camera import Camera


def pick_and_place_arucos():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=45)
    camera.start(name='/IRB140/camera')
    gripper = SuctionPad(simulation=simulation)
    gripper.start()
    # TCP DE LA VENTOSA! OJO!: debe ser el adecuado para la escena
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))

    # Se mueve el robot a una posición articular q
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    # Dos movimientos con MoveJ y MoveL
    robot.moveJ(target_position=Vector([0.4, 0.0, 0.5]),
                target_orientation=Euler([0, np.pi, 0]))
    robot.moveL(target_position=Vector([0.5, 0.0, 0.45]),
                target_orientation=Euler([0, np.pi, 0]))
    # Se averigua T0v para la posición articular actual
    q = robot.get_joint_positions()
    T0v = robot.directkinematics(q)
    # La transformacion entre ventosa y cámara es conocida
    Tvc = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, -np.pi / 2]))
    # Esto permite calcular la transformación entre la cámara y la ARUCO
    id, Tca = camera.detect_closer_aruco(show=True, aruco_size=0.03)
    # La transformación total es:
    T0a = T0v*Tvc*Tca
    frame.show_target_point(target_position=T0a.pos(), target_orientation=T0a.R())
    simulation.wait_time(1)

    simulation.stop()


if __name__ == "__main__":
    pick_and_place_arucos()


