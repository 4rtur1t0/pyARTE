#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_project_ARUCO.ttt scene before running this script.

The code beneath must be completed by the student in order to produce different palletizations of ARUCO pieces
according to their ids.

@Authors: Arturo Gil
@Time: April 2024
"""
import numpy as np
from artelib.euler import Euler
from artelib.path_planning import compute_3D_coordinates
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import get_object_transform, ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
from robots.camera import Camera


def pick_aruco(robot, gripper, frame, tp):
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    Tpick = HomogeneousMatrix(Vector([0, 0, 0]), Euler([0, np.pi, 0]))
    T = tp*Tpick
    frame.show_target_point(T.pos(), T.R())
    gripper.open()
    robot.moveJ(target_position=T.pos(), target_orientation=T.R(), precision=True)
    gripper.close()
    robot.moveAbsJ(q)


def place_aruco(robot, gripper):
    q = np.array([np.pi/2, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    gripper.open()


def find_arucos(robot, camera, frame):
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    robot.moveJ(target_position=Vector([0.5, 0.0, 0.5]),
                target_orientation=Euler([0, np.pi, 0]))
    # transformacion efector (ventosa) cámara
    Tec = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, -np.pi / 2]))
    tps = []
    ids, transformations = camera.detect_arucos(show=True)
    # Encontramos ahora la transformación total hasta las ARUCO
    for i in range(len(ids)):
        print('ID: ', ids[i][0])
        transformations[i].print_nice()
        q = robot.get_joint_positions()
        Te = robot.directkinematics(q)
        Tca = transformations[i]
        Taruco = Te * Tec * Tca
        frame.show_target_point(target_position=Taruco.pos(), target_orientation=Taruco.R())
        tps.append(Taruco)
    return tps


def pick_and_place_arucos():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')
    camera = Camera(simulation=simulation)
    camera.start(name='/IRB140/camera')
    gripper = SuctionPad(simulation=simulation)
    gripper.start()
    # TCP DE LA VENTOSA! OJO!: debe ser el adecuado para la escena
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))
    # Transformacion desde la ventosa al sistema de coordenadas de la cámara
    Tec = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, -np.pi / 2]))

    # Se muestra, a modo informativo, los sistemas de referencia sobre la ventosa (efector final) y sobre la cámara
    q = robot.get_joint_positions()
    T = robot.directkinematics(q)
    # Mostramos el sistema de referencia de la ventosa
    frame.show_target_point(target_position=T.pos(), target_orientation=T.R())
    # Mostramos el sistema de referencia de la cámara
    Tc = T * Tec
    frame.show_target_point(target_position=Tc.pos(), target_orientation=Tc.R())

    simulation.wait(25)
    while True:
        tps = find_arucos(robot, camera, frame)
        if len(tps) == 0:
            break
        for tp in tps:
            pick_aruco(robot, gripper, frame, tp)
            place_aruco(robot, gripper)

    simulation.stop()


if __name__ == "__main__":
    pick_and_place_arucos()


