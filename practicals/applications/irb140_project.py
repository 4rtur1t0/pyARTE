#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_palletizing_color.ttt scene before running this script.

Use the camera on the robot arm to detect a color using a simple image processing.
The mean color of the image is returned and compared to pure colors.
A simple classification in RGB is then used to place the pieces in three main destinations (without any order)

Please: beware that increasing the image resolution may lead to a slow execution.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2, SuctionPad
from robots.objects import Cuboid, ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
from robots.camera import Camera



def get_cuboid_transform(clientID, piece_index):
    """
    Returns the position and orientation of th ith Cuboid in Coppelia
    (in the global reference frame)
    """
    cuboid = Cuboid(clientID)
    if piece_index == 0:
        name = 'Cuboid'
    else:
        name = 'Cuboid' + str(piece_index-1)
    cuboid.start(name)
    p = cuboid.get_position()
    o = cuboid.get_orientation()
    T = HomogeneousMatrix(Vector(p), Euler(o))
    return T



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



def find_color(robot, camera, piece_index):
    """
    Places the camera on top of the piece.
    Saves the image for inspection.
    Computes the image to get the mean RGB value and the color name (red, green, blue).
    """
    T_piece = get_cuboid_transform(clientID=robot.clientID, piece_index=piece_index)
    # leer la posición de la pieza
    p_piece = T_piece.pos()
    to = Euler([np.pi/2, -np.pi, 0])
    # position and orientation so that the camera sees the piece
    tp1 = p_piece + np.array([0.0, 0.0, 0.15])
    # tp2 = p_piece + np.array([0.0, 0.1, 0.2])
    robot.show_target_points([tp1], [to])
    robot.moveJ(target_position=tp1, target_orientation=to)
    # robot.moveL(target_position=tp2, target_orientation=to)
    # capture an image and returns the closest color
    print('get_image')
    color = camera.get_color_name()
    print('Piece is: ', color)
    robot.moveJ(target_position=tp1, target_orientation=to)
    return color


def pick(robot, gripper, piece_index):
    """
    Picks the piece from the conveyor belt.
    """
    T_piece = get_cuboid_transform(clientID=robot.clientID, piece_index=piece_index)
    piece_length = 0.08
    p_piece = T_piece.pos()
    o_piece = T_piece.euler()[0]
    o_piece = o_piece.abg[2]
    tp1 = p_piece + np.array([0, 0, 0.3])
    tp2 = p_piece + np.array([0, -0.2, piece_length/2])
    yaw = -np.pi / 2 - o_piece
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
    to = Euler([0, np.pi, yaw])

    # robot.show_target_points([tp1], [to])
    # robot.show_target_points([tp2], [to])

    gripper.open(precision=False)
    robot.moveJ(target_position=tp1, target_orientation=to, precision='last')
    robot.moveL(target_position=tp2, target_orientation=to, precision='last')
    # for j in range(20):
    while True:
        T_piece = get_cuboid_transform(clientID=robot.clientID, piece_index=piece_index)
        p_piece = T_piece.pos() + np.array([0, 0, piece_length/2])
        e = np.sqrt(np.linalg.norm(tp2-p_piece))
        print('Coupling error e:', e)
        if e < 0.05:
            gripper.close(precision=False)
            break
        else:
            robot.wait()


    gripper.close(precision=False)
    robot.moveL(target_position=tp1, target_orientation=to)


def place(robot, gripper, color, color_indices):
    # define que piece length and a small gap
    piece_length = 0.08
    piece_gap = 0.002

    if color == 'R':
        tp = Vector([-0.6, -0.4, 0.1])  # pallet R base position
        q0 = np.array([-3*np.pi / 4, np.pi / 8, np.pi / 8, 0, -np.pi / 2, -np.pi / 2])
        piece_index = color_indices[0]
        T0m = HomogeneousMatrix(tp, Euler([0, 0, 0]))
    elif color == 'G':
        tp = Vector([-0.2, -0.65, 0.1])  # pallet R base position
        q0 = np.array([-np.pi / 2, np.pi / 8, np.pi / 8, 0, -np.pi / 2, -np.pi / 2])
        piece_index = color_indices[1]
        T0m = HomogeneousMatrix(tp, Euler([0, 0, 0]))
    else:
        tp = Vector([-0.125, 0.75, 0.1])  # pallet R base position
        q0 = np.array([np.pi / 2, np.pi / 8, np.pi / 8, 0, -np.pi / 2, -np.pi / 2])
        piece_index = color_indices[2]
        T0m = HomogeneousMatrix(tp, Euler([0, 0, -np.pi/2]))


    # POSICION DE LA PIEZA i EN EL SISTEMA MÓVIL m (RELATIVA)
    pi = compute_3D_coordinates(index=piece_index, n_x=3, n_y=3, n_z=2, piece_length=piece_length, piece_gap=piece_gap)
    # POSICION p0 INICIAL SOBRE EL PALLET
    p0 = pi + np.array([0, 0, 2.5 * piece_length])
    Tmp0 = HomogeneousMatrix(p0, Euler([0, np.pi, 0]))
    # POSICIÓN p1 EXACTA DE LA PIEZA (considerando su longitud total)
    p1 = pi + np.array([0, 0, piece_length])
    Tmp1 = HomogeneousMatrix(p1, Euler([0, np.pi, 0]))

    # TARGET POINT 0 y 1
    T0 = T0m*Tmp0
    T1 = T0m*Tmp1

    robot.show_target_points([T0.pos()], [T0.R()])
    robot.show_target_points([T1.pos()], [T1.R()])

    robot.moveAbsJ(q0, precision=False)
    robot.moveJ(target_position=T0.pos(), target_orientation=T0.R(), precision='last')
    robot.moveL(target_position=T1.pos(), target_orientation=T1.R(), precision='last')
    gripper.open(precision=True)
    robot.moveL(target_position=T0.pos(), target_orientation=T0.R(), precision='low')


def pick_and_place():
    # Start simulation
    simulation = Simulation()
    clientID = simulation.start()
    # Connect to the robot
    robot = RobotABBIRB140(clientID=clientID)
    robot.start()
    # Connect to the proximity sensor
    conveyor_sensor = ProxSensor(clientID=clientID)
    conveyor_sensor.start()
    # the color camera
    camera = Camera(clientID=clientID)
    camera.start()
    # Connect to the gripper
    # para usar la ventosa
    gripper = SuctionPad(clientID=clientID)
    gripper.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.105]), Euler([-np.pi / 2, 0, 0])))
    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q_target=q0, precision=False)

    piece_index = 0
    color_indices = np.array([0, 0, 0])
    while True:
        T_piece = get_cuboid_transform(clientID=clientID, piece_index=piece_index)
        L = np.sqrt(np.linalg.norm(T_piece.pos()))
        print('Distance to piece is: ', L)
        if L > 0.9:
            robot.wait()
            continue

        color = find_color(robot, camera, piece_index)
        pick(robot, gripper, piece_index)
        place(robot, gripper, color, color_indices)
        robot.moveAbsJ(q_target=q0, precision=False)
        # Next piece!
        piece_index += 1
        if color == 'R':
            color_indices[0] += 1
        elif color == 'G':
            color_indices[1] += 1
        else:
            color_indices[2] += 1

    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

