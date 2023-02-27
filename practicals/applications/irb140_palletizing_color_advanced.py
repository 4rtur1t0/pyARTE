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



def get_cuboid_transform(clientID, index):
    """
    Returns the position and orientation of th ith Cuboid in Coppelia
    """
    cuboid = Cuboid(clientID)
    if index == 0:
        name = 'Cuboid'
    else:
        name = 'Cuboid' + str(index)
    cuboid.start(name)
    p = cuboid.get_position()
    o = cuboid.get_orientation()
    T = HomogeneousMatrix(Vector(p), Euler(o))
    return T


def filter_joint_limits(robot, q):
    n_valid_solutions = q.shape[1]
    q_in_range = []
    for i in range(n_valid_solutions):
        qi = q[:, i]
        total, partial = robot.check_joints(qi)
        if total:
            q_in_range.append(qi)
    q_in_range = np.array(q_in_range).T
    return q_in_range


def get_closest_to(qa, qb):
    """
    From a column wise list of solutions in qa, find the closest to qb.
    """
    n_solutions = qa.shape[1]
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

    q = robot.inversekinematics_line(target_position, target_orientation, vmax=0.5, q0=q0)
    if len(q) == 0:
        print('ERROR: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')

    # EJERCICIO: ELIMINAR ELEMENTOS FUERA DE RANGO
    for i in range(len(q)):
        q[i] = robot.filter_joint_limits(q[i])
    q_traj = []
    for i in range(len(q)):
        try:
            qi = get_closest_to(q[i], q0)
            q0 = qi
            q_traj.append(qi)
        except:
            pass
    q_traj = np.array(q_traj).T
    return q_traj


# def inverse_kinematics(robot, target_position, target_orientation, q0, show_target=True):
#     """
#     Find q that allows the robot to achieve the specified target position and orientaiton
#     CAUTION: target_orientation must be specified as a quaternion in the robot.inversekinematics function
#
#     caution: returns the closest solution to the specified q0
#     """
#     if show_target:
#         frame = ReferenceFrame(clientID=robot.clientID)
#         frame.start()
#         T = HomogeneousMatrix(target_position, target_orientation)
#         frame.set_position_and_orientation(T)
#
#     q = robot.inversekinematics(target_position, target_orientation)
#     if len(q) == 0:
#         print('ERROR: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
#     # filter q according to joint ranges
#     q = robot.filter_joint_limits(q)
#     if len(q) == 0:
#         print('ERROR: NO SOLUTIONS WITHIN RANGE EXIST TO THE INVERSE KINEMATICS')
#     # get closest solution to q0
#     q = get_closest_to(q, q0)
#     return q


def find_color(robot, camera, T_piece):
    """
    Places the camera on top of the piece.
    Saves the image for inspection.
    Computes the image to get the mean RGB value and the color name (red, green, blue).
    """
    # leer la posición de la pieza
    p_piece = T_piece.pos()
    target_orientation = [-np.pi/2, 0, np.pi]
    # position and orientation so that the camera sees the piece
    target_position1 = p_piece + np.array([0.0, 0.1, 0.3])
    target_position2 = p_piece + np.array([0.0, 0.1, 0.2])

    q0 = np.array([0, 0, 0, 0, 0, 0])
    q1 = inverse_kinematics(robot=robot, target_position=target_position1,
                            target_orientation=Euler(target_orientation), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_position2,
                            target_orientation=Euler(target_orientation), q0=q1[:, -1])
    q3 = inverse_kinematics(robot=robot, target_position=target_position1,
                            target_orientation=Euler(target_orientation), q0=q2[:, -1])
    robot.set_joint_target_trajectory(q1, precision='low')
    robot.set_joint_target_trajectory(q2, precision='low')
    # capture an image and returns the closest color
    print('get_image')
    color = camera.get_color_name()
    print('Piece is: ', color)
    robot.set_joint_target_trajectory(q3, precision='low')
    return color


def pick(robot, gripper, T_piece):
    """
    Picks the piece from the conveyor belt.
    """
    piece_length = 0.08
    p_piece = T_piece.pos()
    target_position1 = p_piece + np.array([0, 0, 0.2])
    target_position2 = p_piece + np.array([0, 0, piece_length/2])

    target_orientation = [0, np.pi, -np.pi/2]
    q0 = np.array([0, 0, 0, 0, 0, 0])
    q1 = inverse_kinematics(robot=robot, target_position=target_position1,
                            target_orientation=Euler(target_orientation), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_position2,
                            target_orientation=Euler(target_orientation), q0=q1[:, -1])
    q3 = inverse_kinematics(robot=robot, target_position=target_position1,
                            target_orientation=Euler(target_orientation), q0=q2[:, -1])
    gripper.open(precision=True)
    robot.set_joint_target_trajectory(q1, precision='low')
    robot.set_joint_target_trajectory(q2, precision=True)
    gripper.close(precision=True)
    robot.set_joint_target_trajectory(q3, precision='low')


def place(robot, gripper, color):
    """
    Places, at three different heaps
    """
    if color == 'R':
        target_position = [-0.5, -0.5, 0.6]  # pallet base position
    elif color == 'G':
        target_position = [0.0, -0.5, 0.6]  # pallet base position
    else:
        target_position = [0.5, -0.5, 0.6]  # pallet base position

    base_target_orientation = [0, np.pi, 0]
    q0 = np.array([0, 0, 0, 0, 0, 0])
    q3 = inverse_kinematics(robot=robot, target_position=target_position,
                            target_orientation=Euler(base_target_orientation), q0=q0)
    target_position = target_position + np.array([0, 0, -0.05])
    q4 = inverse_kinematics(robot=robot, target_position=target_position,
                            target_orientation=Euler(base_target_orientation), q0=q0)

    robot.set_joint_target_positions(q3, precision=True)
    robot.set_joint_target_positions(q4, precision=True)
    gripper.open(precision=True)
    robot.set_joint_target_positions(q3, precision=True)


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
    # set the TCP of the suction pad
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.11]), Euler([-np.pi/2, 0, 0])))

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.set_joint_target_positions(q0, precision=True)
    n_pieces = 48
    for i in range(n_pieces):
        while True:
            if conveyor_sensor.is_activated():
                break
            robot.wait()

        T_piece = get_cuboid_transform(clientID=clientID, index=i)

        robot.set_joint_target_positions(q0, precision=True)
        color = find_color(robot, camera, T_piece)
        pick(robot, gripper, T_piece)
        robot.set_joint_target_positions(q0, precision=True)
        place(robot, gripper, color)

    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

