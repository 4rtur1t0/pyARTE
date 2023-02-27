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
        # pxyz.append([j[n], k[n], i[n]])
        pxyz.append([j[n]*dxy, k[n]*dxy, i[n]*dz])
    pxyz = np.array(pxyz)

    if index < n_z * n_x * n_y:
        return pxyz[index, :]
    else:
        print('CAUTION: N PIECES IS LARGER THAN NX*NY*NZ')
        return pxyz[-1, :]


def get_closest_to(qa, qb):
    """
    From a column wise list of solutions in qa, find the closest to qb.
    """
    if len(qa) > 0:
        n_solutions = qa.shape[1]
    else:
        return []
    distances = []
    for i in range(n_solutions):
        d = np.linalg.norm(qa[:, i]-qb)
        distances.append(d)
    distances = np.array(distances)
    distances = np.nan_to_num(distances, nan=np.inf)
    idx = np.argmin(distances)
    return qa[:, idx]


def inverse_kinematics(robot, target_position, target_orientation, q0, show_target=True):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion in the robot.inversekinematics function

    caution: returns the closest solution to the specified q0
    """
    if show_target:
        frame = ReferenceFrame(clientID=robot.clientID)
        frame.start()
        T = HomogeneousMatrix(target_position, target_orientation)
        frame.set_position_and_orientation(T)

    q = robot.inversekinematics(target_position, target_orientation)
    if len(q) == 0:
        print('ERROR: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
    # filter q according to joint ranges
    q = robot.filter_joint_limits(q)
    if len(q) == 0:
        print('ERROR: NO SOLUTIONS WITHIN RANGE EXIST TO THE INVERSE KINEMATICS')
    # get closest solution to q0
    q = get_closest_to(q, q0)
    return q


def pick(robot, gripper):
    q0 = np.array([0, 0, 0, 0, 0, 0])
    target_positions = [[0.6, 0.267, 0.23],  # approximation
                        [0.6, 0.267, 0.19]] # pick
    target_orientations = [Euler([0, np.pi, 0]),
                           Euler([0, np.pi, 0])]
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=target_orientations[0], q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=target_orientations[1], q0=q1)
    gripper.open(precision=True)
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    gripper.close(precision=True)
    robot.set_joint_target_positions(q1, precision=True)


def place(robot, gripper, i):
    # define que piece length and a small gap
    piece_length = 0.08
    piece_gap = 0.02
    q0 = np.array([0, 0, 0, 0, 0, 0])

    # POSICION BASE DEL PALLET A LA DERECHA DEL ROBOT
    base_target_position = [-0.15, -0.65, 0.1]
    base_target_orientation = Euler([0, 0, 0])
    # POSICION BASE DEL PALLET A LA IZQUIERDA DEL ROBOT
    # base_target_position = [-0.07, 0.36, 0.1]  # pallet 2 base position
    # base_target_orientation = Euler([0, 0, np.pi/6])
    T0m = HomogeneousMatrix(base_target_position, base_target_orientation)
    # POSICION DE LA PIEZA i EN EL SISTEMA MÓVIL m
    p = compute_3D_coordinates(index=i, n_x=3, n_y=4, n_z=2, piece_length=piece_length, piece_gap=piece_gap)
    # POSICION INICIAL SOBRE EL PALLET
    p0 = p + np.array([0, 0, 2.5 * piece_length])
    # POSICIÓN EXACTA DE LA PIEZA (considerando la mitad de su lado)
    p1 = p + np.array([0, 0, 0.5 * piece_length])
    # Calculamos la transformación relativa en el sistema de referencia del pallet
    Tmp = HomogeneousMatrix(p0, Euler([0, np.pi, 0]))
    T = T0m * Tmp
    q0 = inverse_kinematics(robot=robot, target_position=T.pos(), target_orientation=T.R(), q0=q0)
    Tmp = HomogeneousMatrix(p1, Euler([0, np.pi, 0]))
    T = T0m*Tmp
    q1 = inverse_kinematics(robot=robot, target_position=T.pos(), target_orientation=T.R(), q0=q0)

    # EJECUTAMOS LOS MOVIMIENTOS
    robot.set_joint_target_positions(q0, precision=True)
    robot.set_joint_target_positions(q1, precision=True)
    gripper.open(precision=True)
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q0, precision=True)


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


    # Connect to the gripper
    gripper = GripperRG2(clientID=clientID)
    gripper.start()
    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))

    # para usar la ventosa
    # gripper = SuctionPad(clientID=clientID)
    # gripper.start()
    # # set the TCP of the suction pad
    # robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.11]), Euler([-np.pi/2, 0, 0])))

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    n_pieces = 24
    for i in range(n_pieces):
        while True:
            if conveyor_sensor.is_activated():
                break
            simulation.wait()
        robot.set_joint_target_positions(q0, precision=True)
        pick(robot, gripper)
        robot.set_joint_target_positions(q0, precision=True)
        place(robot, gripper, i)
    # Stop arm and simulation
    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

