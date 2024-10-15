#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_project_ARUCO.ttt scene before running this script.

The code beneath must be completed by the student in order to produce different palletizations of ARUCO pieces
according to their ids.

@Authors: Arturo Gil
@Time: February 2024
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
import matplotlib.pyplot as plt

def look_for_arucos(robot, camera, show=True, aruco_size=0.1):
    """
    In case we want a list to all ARUCOS along with its transformation
    """
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    robot.moveJ(target_position=Vector([0.5, 0.05, 0.35]),
                target_orientation=Euler([0, np.pi, 0]))
    id, Tca = camera.detect_closer_aruco(show=show, aruco_size=aruco_size)
    return id, Tca


def compute_target_point(robot, frame, Tca):
    """
    Compute the full transformationIn case we want a list to all ARUCOS along with its transformation
    """

    # Transformación base --> ventosa (incluye TCP)
    q = robot.get_joint_positions()
    Tv = robot.directkinematics(q)
    Tec = HomogeneousMatrix([[0., 1., 0., 0.],
                             [-1., 0., 0., -0.05],
                             [0., 0., 1., 0.],
                             [0., 0., 0., 1.]])
    # transformacion ventosa--> cámara
    # Tec = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, -np.pi / 2]))
    # transformacion total
    T = Tv*Tec*Tca
    frame.show_target_point(target_position=T.pos(), target_orientation=T.R())
    robot.wait(1)
    frame.show_target_point(target_position=np.array([0, 0 , 0]), target_orientation=T.R())
    return T


def observe_arucos():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=25.0)
    camera.start(name='/IRB140/camera')
    gripper = SuctionPad(simulation=simulation)
    gripper.start()
    # TCP DE LA VENTOSA! OJO!: debe ser el adecuado para la escena
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))

    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)

    tp = np.array([0.5, 0, 0.05])
    es = []
    zs = []
    for i in range(15):
        z = 0.7-0.02*i
        robot.moveJ(target_position=Vector([0.5, 0.05, z]),
                    target_orientation=Euler([0, np.pi, 0]), endpoint=True,
                    precision=True)
        simulation.wait()
        id, Tca = camera.detect_closer_aruco(show=False)
        if id is None:
            print('NO ARUCO FOUND')
            break
        # Compute the target point considering all the transformations
        T = compute_target_point(robot, frame, Tca)

        print('Observación: ', i)
        print(T.pos())
        print(T.R().euler()[0])
        es.append(tp-T.pos())
        zs.append(z)

    plt.plot(zs, es)
    plt.show()
    simulation.stop()


if __name__ == "__main__":
    observe_arucos()

