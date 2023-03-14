#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_suction_pad.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation


def compute_3D_coordinates(index, n_x, n_y, n_z, piece_length, piece_gap):
    """
    Compute 3D coordinates for cubic pieces in a 3D array.
    """
    dxy = piece_length + piece_gap
    dz = piece_length
    # get the indices of a n_i xn_j x n_k array
    i, j, k = np.indices((n_z, n_x, n_y))
    i = i.flatten()
    j = j.flatten()
    k = k.flatten()
    pxyz = []
    for n in range(n_z * n_x * n_y):
        pxyz.append([j[n]*dxy, k[n]*dxy, i[n]*dz])
    pxyz = np.array(pxyz)
    if index < n_z * n_x * n_y:
        return pxyz[index, :]
    else:
        print('WARNING: N PIECES IS LARGER THAN NX*NY*NZ')
        index = index - n_z * n_x * n_y
        return pxyz[index, :]


def pick(robot, gripper):
    tp1 = Vector([0.6, 0.265, 0.45]) # approximation
    tp2 = Vector([0.6, 0.265, 0.193+0.04]) # pick
    to = Euler([0, np.pi, -np.pi/2])
    # q0 = np.array([0, 0, 0, 0, 0, 0])
    robot.show_target_points([tp2], [to])
    gripper.open(precision=True)
    robot.moveJ(target_position=tp1, target_orientation=to)
    robot.moveL(target_position=tp2, target_orientation=to, precision='last')
    gripper.close(precision=True)
    robot.moveL(target_position=tp1, target_orientation=to)


def place(robot, gripper, i):
    # define que piece length and a small gap
    piece_length = 0.08
    piece_gap = 0.003
    q0 = np.array([-np.pi/4, 0, 0, 0, 0.1, -np.pi/2])
    # POSITION AND ORIENTATION OF THE PALLET
    T0m = HomogeneousMatrix(Vector([-0.15, -0.7, 0.1]), Euler([0, 0, 0]))
    # POSICION DE LA PIEZA i EN EL SISTEMA MÓVIL m (RELATIVA)
    pi = compute_3D_coordinates(index=i, n_x=3, n_y=2, n_z=3, piece_length=piece_length, piece_gap=piece_gap)
    # POSICION p0 INICIAL SOBRE EL PALLET
    p0 = pi + np.array([0, 0, 4 * piece_length])
    Tmp0 = HomogeneousMatrix(p0, Euler([0, np.pi, 0]))
    # POSICIÓN p1 EXACTA DE LA PIEZA (considerando su longitud total)
    p1 = pi + np.array([0, 0, 1.01*piece_length])
    Tmp1 = HomogeneousMatrix(p1, Euler([0, np.pi, 0]))

    # TARGET POINT 0 y 1
    T0 = T0m*Tmp0
    T1 = T0m*Tmp1

    robot.show_target_points([T1.pos()], [T1.euler()[0]])

    robot.moveAbsJ(q0, precision=True)
    robot.moveJ(target_position=T0.pos(), target_orientation=T0.R(), precision='last')
    robot.moveL(target_position=T1.pos(), target_orientation=T1.R(), precision='last')
    gripper.open(precision=True)
    robot.moveL(target_position=T0.pos(), target_orientation=T0.R(), precision='last')


def pick_and_place():
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotABBIRB140(clientID=clientID)
    robot.start()
    conveyor_sensor = ProxSensor(clientID=clientID)
    conveyor_sensor.start()
    gripper = SuctionPad(clientID=clientID)
    gripper.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.105]), Euler([-np.pi / 2, 0, 0])))
    q0 = np.array([0, 0, 0, 0, 0.1, -np.pi/2])
    robot.moveAbsJ(q_target=q0)
    n_pieces = 18
    for i in range(n_pieces):
        while True:
            if conveyor_sensor.is_activated():
                break
            robot.wait()
        robot.moveAbsJ(q_target=q0)
        pick(robot, gripper)
        robot.moveAbsJ(q_target=q0)
        place(robot, gripper, i)
    print('FINISHED!')
    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

