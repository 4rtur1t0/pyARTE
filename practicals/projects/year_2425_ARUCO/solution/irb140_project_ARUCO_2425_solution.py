#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_project_ARUCO_2425.ttt scene before running this script.

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


def find_aruco_transform(robot, camera):
    id, T0p = look_for_aruco(robot, camera, show=True, aruco_size=0.03)

    robot.moveJ(target_position=T0p.pos() + np.array([0.0, 0.05, 0.1]),
                target_orientation=Euler([0, np.pi, 0]))
    id, T0p = look_for_aruco(robot, camera, show=True, aruco_size=0.03)
    return id, T0p


def look_for_aruco(robot, camera, show=True, aruco_size=0.03):
    """
    In case we want a list to all ARUCOS along with its transformation
    """
    id, Tca = camera.detect_closer_aruco(show=show, aruco_size=aruco_size)
    if id is None:
        print('could not find piece')
        return None, None
    q = robot.get_joint_positions()
    Te = robot.directkinematics(q)
    # transformacion ventosa--> cámara
    Tec = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, -np.pi / 2]))
    # transformacion total
    T = Te * Tec * Tca
    return id, T


def pick_piece_from_pallet(robot, gripper, frame, Tpiece):
    Tpick = HomogeneousMatrix(Vector([0, 0, 0]), Euler([0, np.pi, 0]))
    T = Tpiece*Tpick
    frame.show_target_point(T.pos(), T.R())
    gripper.open()
    print('move1')
    robot.moveL(target_position=T.pos()+np.array([0, 0, +0.05]), target_orientation=T.R(), vmax=1.0, precision=False)
    print('move2')
    robot.moveL(target_position=T.pos()+np.array([0, 0, +0.001]), target_orientation=T.R(), precision=True, vmax=0.05)
    gripper.close()
    print('move3')
    robot.moveL(target_position=T.pos()+np.array([0, 0, 0.1]), target_orientation=T.R(), precision=False, vmax=1.2)
    frame.show_target_point(Vector([0, 0, 0]), T.R())
    print('move4')


def place_piece_on_frame(robot, gripper, frame, id, T0f):
    Tpick = HomogeneousMatrix(Vector([0, 0, 0]), Euler([0, np.pi, 0]))
    print('PLACING PIECE WITH ID: ', id)
    # Piece I
    if id == 1:
        Tfp = HomogeneousMatrix(Vector([0.05, 0.125, 0]), Euler([0, 0, 0]))
        T0p = T0f*Tfp*Tpick
    # Piece T
    elif id == 2: #0.05
        Tfp = HomogeneousMatrix(Vector([0.2, 0.225, 0]), Euler([0, 0, 0]))
        T0p = T0f*Tfp*Tpick
    # Piece L
    elif id == 3:
        Tfp = HomogeneousMatrix(Vector([0.205, 0.1, 0]), Euler([0, 0, 0]))
        T0p = T0f*Tfp*Tpick
    # Piece 0
    elif id == 4:
        Tfp = HomogeneousMatrix(Vector([0.325, 0.175, 0]), Euler([0, 0, 0]))
        T0p = T0f * Tfp * Tpick

    frame.show_target_point(T0p.pos(), T0p.R())
    robot.moveJ(target_position=T0p.pos() + np.array([0, 0, +0.05]), target_orientation=T0p.R(), qdfactor=1.0, precision=False)
    robot.moveL(target_position=T0p.pos() + np.array([0, 0, +0.01]), target_orientation=T0p.R(), precision=True, vmax=0.05)
    gripper.open()
    robot.moveL(target_position=T0p.pos() + np.array([0, 0, 0.1]), target_orientation=T0p.R(), precision=True, vmax=1.0)
    # robot.moveAbsJ(q)




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
    q = np.array([np.pi/2, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    # find frame
    # _, T0f = look_for_frame(robot, camera, show=True, aruco_size=0.03)
    robot.moveJ(target_position=Vector([0.1, 0.35, 0.5]),
                target_orientation=Euler([0, np.pi, 0]))
    _, T0f = find_aruco_transform(robot, camera)
    frame.show_target_point(target_position=T0f.pos(), target_orientation=T0f.R())
    # simulation.wait_time(seconds=2)
    for i in range(4):
        robot.moveJ(target_position=Vector([0.5, 0.0, 0.6]),
                    target_orientation=Euler([0, np.pi, 0]))
        id, T0p = find_aruco_transform(robot, camera)
        if id is None:
            print('NO ARUCO FOUND')
            break

        # Compute the target point considering all the transformations
        pick_piece_from_pallet(robot, gripper, frame, T0p)
        place_piece_on_frame(robot, gripper, frame, id, T0f)

    simulation.stop()


if __name__ == "__main__":
    pick_and_place_arucos()


