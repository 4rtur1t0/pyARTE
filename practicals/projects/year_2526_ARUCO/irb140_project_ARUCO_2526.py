#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_project_2526.ttt scene before running this script.

@Authors: Arturo Gil
@Time: February 2025
"""
import numpy as np
from artelib.euler import Euler
from artelib.path_planning import compute_3D_coordinates
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import get_object_transform, ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
from robots.camera import Camera


def move_to_pos_B(robot):
    """
    Se mueve al robot a una zona donde se pueden visualizar las piezas rojas
    desde arriba.
    """
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    tpos = Vector([0.2, 0.5, 0.4])
    torient = Euler([0, np.pi, 0])
    robot.moveJ(target_position=tpos,
                target_orientation=torient)


def pick_and_place_arucos():
    """
    Se proporcionan las clases necesarias y las funciones mínimas para abordar el proyecto.
    """
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=75.0)
    camera.start(name='/IRB140/camera')
    gripper = SuctionPad(simulation=simulation)
    gripper.start()
    # TCP DE LA VENTOSA! OJO!: debe ser el adecuado para la escena
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))
    # un contador de piezas en funcion de su identificador
    piece_counter = {0: -1, 1: -1, 2: -1, 3: -1}

    # Se proporcionan, a continuación, algunas funciones útiles
    # para resolver el proyecto.
    move_to_pos_B(robot)
    # Transformación base --> ventosa (incluye TCP)
    Tv = robot.directkinematics(robot.get_joint_positions())
    # Transformación ventosa--> cámara
    Tvc = HomogeneousMatrix(Vector([0, -0.05, 0]), Euler([0, 0, -np.pi / 2]))
    # Detección y cálculo de las ARUCOS
    id_aruco, Tca = camera.detect_closer_aruco(show=True, aruco_size=0.02)
    # Cálculo de la transformación total (La ARUCO en coordenadas de la BASE)
    Tt = Tv * Tvc * Tca
    # Se muestra la posicion/orientación observadas/calculadas en Coppelia
    frame.show_target_point(target_position=Tt.pos(),
                            target_orientation=Tt.R())
    # El contador de piezas se actualiza así:
    piece_counter[id_aruco] = piece_counter.get(id_aruco, None) + 1
    simulation.wait_time(3)
    simulation.stop()


if __name__ == "__main__":
    pick_and_place_arucos()


