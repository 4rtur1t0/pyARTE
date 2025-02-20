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


def look_for_arucos(robot, camera, show=True, aruco_size=0.1):
    """
    In case we want a list to all ARUCOS along with its transformation
    """
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    # robot.moveJ(target_position=Vector([0.5, 0.05, 0.35]),
    #             target_orientation=Euler([0, np.pi, 0]))
    id, Tca = camera.detect_closer_aruco(show=show, aruco_size=0.1)
    return id, Tca


def calibrate(robot, camera):
    """
    Must be used to compute the transformation between the suction cap and the camera.
    The position and orientation of the reference system captured by the camera on the ARUCO must be known.
    Place the ARUCO on a known position and orientation and make the camera look at it from a clear vantage point.
    """
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    robot.moveJ(target_position=Vector([0.5, 0.05, 0.35]),
                target_orientation=Euler([0, np.pi, 0]))
    id, Tca = camera.detect_closer_aruco(show=show)
    q = robot.get_joint_positions()
    T = robot.directkinematics(q)
    # KNOWN POSITION/ORIENTATION OF THE ARUCO
    T0A = HomogeneousMatrix(Vector([0.5, 0.0, 0.07]), Euler([0, 0, 0]))
    # obtain the relative transformation
    Tvc = T.inv()*T0A*Tca.inv()
    Tvc.print_nice()
    return Tvc


def compute_target_point(robot, frame, Tca):
    """
    Compute the full transformationIn case we want a list to all ARUCOS along with its transformation
    """
    # Tec = HomogeneousMatrix([[0.,    1.,    0.,    0.],
    #                      [-1.,    0.,    0.,   -0.05],
    #                      [0.,    0.,    1.,    0.],
    #                      [0.,    0.,    0.,    1.]])
    # Transformación base --> ventosa (incluye TCP)
    q = robot.get_joint_positions()
    Te = robot.directkinematics(q)
    # transformacion ventosa--> cámara
    Tec = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, -np.pi / 2]))
    # transformacion total
    T = Te*Tec*Tca
    frame.show_target_point(target_position=T.pos(), target_orientation=T.R())
    return T


def pick_aruco(robot, gripper, frame, tp):
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    Tpick = HomogeneousMatrix(Vector([0, 0, 0]), Euler([0, np.pi, 0]))
    T = tp*Tpick
    frame.show_target_point(T.pos(), T.R())
    gripper.open()
    robot.moveL(target_position=T.pos()+np.array([0, 0, +0.1]), target_orientation=T.R(), vmax=1.0, precision=False)
    robot.moveL(target_position=T.pos(), target_orientation=T.R(), precision=True, vmax=0.2)
    gripper.close()
    robot.moveL(target_position=T.pos()+np.array([0, 0, 0.2]), target_orientation=T.R(), precision=True)
    robot.moveAbsJ(q)


def place_aruco(robot, gripper):
    q = np.array([np.pi/2, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    gripper.open()


def pick_and_place_arucos():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    # conveyor_sensor = ProxSensor(simulation=simulation)
    # conveyor_sensor.start(name='/conveyor/prox_sensor')
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=45)
    camera.start(name='/IRB140/camera')
    gripper = SuctionPad(simulation=simulation)
    gripper.start()
    # TCP DE LA VENTOSA! OJO!: debe ser el adecuado para la escena
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))
    # q = robot.get_joint_positions()
    # T0 = robot.directkinematics(q)

    simulation.wait_time(seconds=2)

    for i in range(6):
        # frame.show_target_point(target_position=T0.pos(), target_orientation=T0.R())
        # while True:
        #     simulation.wait()
        #     if conveyor_sensor.is_activated():
        #         break
        id, Tca = look_for_arucos(robot, camera, show=True, aruco_size=0.1)

        if id is None:
            print('NO ARUCO FOUND')
            break
        # Compute the target point considering all the transformations
        tp = compute_target_point(robot, frame, Tca)
        pick_aruco(robot, gripper, frame, tp)
        place_aruco(robot, gripper)
        simulation.wait()

    simulation.stop()


if __name__ == "__main__":
    pick_and_place_arucos()


