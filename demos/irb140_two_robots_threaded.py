#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/irb140_two_robots.ttt scene before running this script.

Two abb robots and a pick and place application with palletizing.

In this version, python Threads are used so that both robots can be used independently.

CAUTION: One of the two proximity sensors shows an erratic beheviour at the beginning of the simulation.

@Authors: Arturo Gil
@Time: April 2022
"""
import threading

import numpy as np
from artelib.euler import Euler
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad, GripperRG2
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation

global clientID

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
    # filter in joint ranges
    q = filter_joint_limits(robot, q)
    # get closest solution to q0
    q = get_closest_to(q, q0)
    return q


def pick1(robot, gripper):
    target_positions = [[0.43, -0.28, 0.45],  # approximation
                        [0.43, -0.28, 0.35]] # pick
    target_orientations = [[0, np.pi, 0],
                           [0, np.pi, 0]]
    q0 = np.array([0, 0, 0, 0, 0, 0])
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=Euler(target_orientations[0]), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=Euler(target_orientations[1]), q0=q0)

    gripper.open(precision=True)
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    gripper.close(precision=True)
    robot.set_joint_target_positions(q1, precision=True)


def place1(robot, gripper, i):
    base_target_positions = [[-0.1, -0.65, 0.45],
                             [-0.1, -0.65, 0.4]]
    base_target_orientation = [0, np.pi, 0]
    q0 = np.array([0, 0, 0, 0, 0, 0])

    q1 = inverse_kinematics(robot=robot, target_position=base_target_positions[0],
                            target_orientation=Euler(base_target_orientation), q0=q0)

    q2 = inverse_kinematics(robot=robot, target_position=base_target_positions[1],
                            target_orientation=Euler(base_target_orientation), q0=q0)

    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    gripper.open(precision=True)
    robot.set_joint_target_positions(q1, precision=True)


def pick2(robot, gripper):
    target_positions = [[0.5, 0.0, 0.45],  # approximation
                        [0.5, 0.0, 0.23]] # pick
    target_orientations = [[0, np.pi, np.pi/2],
                           [0, np.pi, np.pi/2]]
    q0 = np.array([0, 0, 0, 0, 0, 0])
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=Euler(target_orientations[0]), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=Euler(target_orientations[1]), q0=q0)

    gripper.open(precision=True)
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    gripper.close(precision=True)
    robot.set_joint_target_positions(q1, precision=True)


def place2(robot, gripper, i):
    base_target_positions = [[-0.1, -0.6, 0.45],
                             [-0.1, -0.6, 0.4]]
    base_target_orientation = [0, np.pi, 0]
    q0 = np.array([0, 0, 0, 0, 0, 0])

    q1 = inverse_kinematics(robot=robot, target_position=base_target_positions[0],
                            target_orientation=Euler(base_target_orientation), q0=q0)

    q2 = inverse_kinematics(robot=robot, target_position=base_target_positions[1],
                            target_orientation=Euler(base_target_orientation), q0=q0)

    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    gripper.open(precision=True)
    robot.set_joint_target_positions(q1, precision=True)


def pick_and_place1():

    global clientID
    robot1 = RobotABBIRB140(clientID=clientID)
    robot1.start(base_name='/IRB140')

    conveyor_sensor1 = ProxSensor(clientID=clientID)
    conveyor_sensor1.start(name='/conveyor/prox_sensor')

    gripper1 = GripperRG2(clientID=clientID)
    gripper1.start(name='/IRB140/RG2/RG2_openCloseJoint')

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot1.set_joint_target_positions(q0, precision=True)

    n_pieces = 27
    for i in range(n_pieces):
        while True:
            if conveyor_sensor1.is_activated():
                break
            simulation.wait()
        robot1.set_joint_target_positions(q0, precision=True)
        pick1(robot1, gripper1)
        robot1.set_joint_target_positions(q0, precision=True)
        place1(robot1, gripper1, i)




def pick_and_place2():
    global clientID

    robot2 = RobotABBIRB140(clientID=clientID)
    robot2.start(base_name='/IRB140_2')

    conveyor_sensor2 = ProxSensor(clientID=clientID)
    conveyor_sensor2.start(name='/conveyor_2/prox_sensor_2')

    gripper2 = SuctionPad(clientID=clientID)
    gripper2.start()

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot2.set_joint_target_positions(q0, precision=True)

    n_pieces = 27
    for i in range(n_pieces):
        while True:
            if conveyor_sensor2.is_activated():
                break
            simulation.wait()
        robot2.set_joint_target_positions(q0, precision=True)
        pick2(robot2, gripper2)
        robot2.set_joint_target_positions(q0, precision=True)
        place2(robot2, gripper2, i)




if __name__ == "__main__":
    global clientID
    simulation = Simulation()
    clientID = simulation.start()

    r1 = threading.Thread(target=pick_and_place1, args=())
    r2 = threading.Thread(target=pick_and_place2, args=())

    r1.start()
    r2.start()

    # while True:
    #     if fin1 and fin2:
    #         print('DONE closing connection')
    #         # Stop arm and simulation
    #         robot1.stop_arm()
    #         robot1.stop_simulation()
    #         break

    # simulation.stop()
