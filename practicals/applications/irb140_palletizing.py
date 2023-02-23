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
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation


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


def inverse_kinematics(robot, target_position, target_orientation, q0):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion in the robot.inversekinematics function

    caution: closest tooooo
    """
    q = robot.inversekinematics(target_position, target_orientation)
    if len(q) == 0:
        print('ERROR: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
    # filter in joint ranges
    q = filter_joint_limits(robot, q)
    if len(q) == 0:
        print('ERROR: NO SOLUTIONS WITHIN RANGE EXIST TO THE INVERSE KINEMATICS')
    # get closest solution to q0
    q = get_closest_to(q, q0)
    return q


def pick(robot, gripper):
    # if using the RG2 gripper
    target_positions = [[0.6, 0.267, 0.25],  # approximation
                        [0.6, 0.267, 0.19]] # pick
    target_orientations = [[0, np.pi, 0],
                           [0, np.pi, 0]]

    # WHEN USING THE SUCTION PAD!!!
    # target_positions = [[0.6, 0.267, 0.45],  # approximation
    #                     [0.6, 0.267, 0.23]]  # pick
    # target_orientations = [[0, np.pi, -np.pi/2],
    #                        [0, np.pi, -np.pi/2]]

    q0 = np.array([0, 0, 0, 0, 0, 0])
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=Euler(target_orientations[0]), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=Euler(target_orientations[1]), q0=q1)

    gripper.open(precision=True)
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    gripper.close(precision=True)
    robot.set_joint_target_positions(q1, precision=True)


def place(robot, gripper, i):
    q0 = np.array([0, 0, 0, 0, 0, 0])
    # pallet approximation
    pallet_target_position = [-0.2, -0.65, 0.5]  # pallet base position
    pallet_target_orientation = [0, np.pi, 0]

    q0 = inverse_kinematics(robot=robot, target_position=pallet_target_position,
                            target_orientation=Euler(pallet_target_orientation), q0=q0)

    robot.set_joint_target_positions(q0, precision=True)
    # when using the RG2 gripper
    base_target_position = [-0.2, -0.65, 0.22]  # pallet base position
    base_target_orientation = [0, np.pi, 0]

    # when using the Suction pad
    base_target_position = [-0.2, -0.7, 0.25]  # pallet base position
    base_target_orientation = [0, np.pi, 0]

    # i, j = np.indices((3, 4))
    # print(i)
    # print(j)
    # i = i.flatten()
    # j = j.flatten()
    # v = []
    # for k in range(3*4):
    #     v.append([i[k], j[k]])

    n_i = 2
    n_j = 3
    n_k = 4

    i, j, k = np.indices((n_i, n_j, n_k))
    print(i)
    print(j)
    i = i.flatten()
    j = j.flatten()
    k = k.flatten()
    v = []
    for n in range(n_i*n_j*n_k):
        # v.append([i[n], j[n], k[n]])
        v.append([j[n], k[n], i[n]])

    # define que piece length and a small ga
    piece_length = 0.08
    # a gap between pieces
    piece_gap = 0.01

    n = 3 # n rows n columns
    kx = i % n
    ky = np.floor((i / n) % n)
    kz = np.floor(i / (n * n))
    target_position = base_target_position + kx*np.array([piece_length+piece_gap, 0, 0]) + \
                      ky*np.array([0, piece_length+piece_gap, 0]) + \
                      kz*np.array([0, 0, piece_length]) + np.array([0, 0, piece_gap])
    q1 = inverse_kinematics(robot=robot, target_position=target_position,
                            target_orientation=Euler(base_target_orientation), q0=q0)
    target_position = target_position + np.array([0, 0, -piece_length])
    q2 = inverse_kinematics(robot=robot, target_position=target_position,
                            target_orientation=Euler(base_target_orientation), q0=q0)

    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    gripper.open(precision=True)
    robot.set_joint_target_positions(q1, precision=True)


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

    n_pieces = 48
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

