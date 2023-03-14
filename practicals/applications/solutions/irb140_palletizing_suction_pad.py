#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.vector import Vector
from artelib.rotationmatrix import RotationMatrix
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2, SuctionPad
from robots.objects import ReferenceFrame
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
    q0 = np.array([np.pi/4, np.pi/8, np.pi/8, 0, -np.pi/2, -np.pi/2])
    tp1 = Vector([0.6, 0.267, 0.30])  # approximation
    tp2 = Vector([0.6, 0.267, 0.225]) # pick
    to = Euler([0, np.pi, -np.pi/2])

    robot.show_target_points([tp1], [to])
    robot.show_target_points([tp2], [to])

    robot.moveAbsJ(q0, precision=True)
    gripper.open(precision=True)
    robot.moveJ(target_position=tp1, target_orientation=to, precision=False)
    robot.moveL(target_position=tp2, target_orientation=to, precision=True)
    gripper.close(precision=True)
    robot.moveL(target_position=tp1, target_orientation=to, precision=False)


def place(robot, gripper, i):
    # define que piece length and a small gap
    piece_length = 0.08
    piece_gap = 0.002
    q0 = np.array([-np.pi/4, np.pi/8, np.pi/8, 0, -np.pi/2, -np.pi/2])
    # POSITION AND ORIENTATION OF THE PALLET
    T0m = HomogeneousMatrix(Vector([-0.15, -0.7, 0.14]), Euler([0, 0, 0]))
    # POSICION DE LA PIEZA i EN EL SISTEMA MÓVIL m (RELATIVA)
    pi = compute_3D_coordinates(index=i, n_x=3, n_y=3, n_z=2, piece_length=piece_length, piece_gap=piece_gap)
    # POSICION p0 INICIAL SOBRE EL PALLET
    p0 = pi + np.array([0, 0, 2.5 * piece_length])
    Tmp0 = HomogeneousMatrix(p0, Euler([0, np.pi, 0]))
    # POSICIÓN p1 EXACTA DE LA PIEZA (considerando la mitad de su lado)
    p1 = pi + np.array([0, 0, 0.5 * piece_length])
    Tmp1 = HomogeneousMatrix(p1, Euler([0, np.pi, 0]))

    # TARGET POINT 0 y 1
    T0 = T0m*Tmp0
    T1 = T0m*Tmp1

    robot.show_target_points([T0.pos()], [T0.R()])
    robot.show_target_points([T1.pos()], [T1.R()])

    robot.moveAbsJ(q0, precision=True)
    robot.moveJ(target_position=T0.pos(), target_orientation=T0.R(), precision='last')
    robot.moveL(target_position=T1.pos(), target_orientation=T1.R(), precision=True)
    gripper.open(precision=True)
    robot.moveL(target_position=T0.pos(), target_orientation=T0.R(), precision='low')


def pick_and_place():
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotABBIRB140(clientID=clientID)
    robot.start()
    conveyor_sensor = ProxSensor(clientID=clientID)
    conveyor_sensor.start()

    gripper = SuctionPad(clientID=clientID)
    gripper.start()
    # set the TCP of the suction pad
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.105]), Euler([-np.pi/2, 0, 0])))

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q0, precision=True)
    n_pieces = 24
    for i in range(n_pieces):
        while True:
            if conveyor_sensor.is_activated():
                break
            simulation.wait()
        pick(robot, gripper)
        place(robot, gripper, i)
    # Stop arm and simulation
    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

