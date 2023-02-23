#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
# from sceneconfig.init_sim import init_sim

from robots.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2, SuctionPad
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
# from sceneconfig.scene_configs_irb140 import init_simulation_ABBIRB140


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


def pick(robot, gripper):
    target_positions = [[0.55, 0.265, 0.55],  # approximation
                        [0.55, 0.265, 0.295]]  # pick
    target_orientations = [[0, np.pi/2, -np.pi/2],
                           [0, np.pi/2, -np.pi/2]]

    q0 = np.array([0, 0, 0, 0, 0, 0])
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=Euler(target_orientations[0]), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=Euler(target_orientations[1]), q0=q1)

    gripper.suction_off()
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    gripper.suction_on()
    robot.set_joint_target_positions(q1, precision=True)


def place(robot, gripper, i):
    q0 = np.array([0, 0, 0, 0, 0, 0])
    base_target_position = [-0.2, -0.65, 0.32]  # pallet base position
    base_target_orientation = [np.pi/2, 0, np.pi]

    # define que piece length and a small gap
    piece_length = 0.08
    # a gap between pieces
    piece_gap = 0.005
    # calculamos las constantes kx, ky y kz para obtener la posicion
    n = 3 # n rows n columns
    kx = i % n
    ky = np.floor((i / n) % n)
    kz = np.floor(i / (n * n))
    # 0.5 m above
    target_position = base_target_position + kx*np.array([piece_length+piece_gap, 0, 0]) + \
                     ky*np.array([0, piece_length+piece_gap, 0]) + \
                     kz*np.array([0, 0, piece_length]) + np.array([0, 0, piece_gap]) + \
                     np.array([0, 0, 0.3])

    q1 = inverse_kinematics(robot=robot, target_position=target_position,
                            target_orientation=Euler(base_target_orientation), q0=q0)

    target_position = target_position - np.array([0, 0, 0.3])

    q2 = inverse_kinematics(robot=robot, target_position=target_position,
                            target_orientation=Euler(base_target_orientation), q0=q0)
    target_position = target_position + np.array([0, 0, -piece_length])
    q3 = inverse_kinematics(robot=robot, target_position=target_position,
                            target_orientation=Euler(base_target_orientation), q0=q0)

    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    robot.set_joint_target_positions(q3, precision=True)
    gripper.suction_off()
    robot.set_joint_target_positions(q2, precision=True)


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

    # or, alternatively use the SuctioPad
    # if using the SuctionPad, the target position and orientations should be changed in pick and place
    gripper = SuctionPad(clientID=clientID)
    gripper.start()

    # robot, conveyor_sensor = init_simulation_ABBIRB140()
    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.set_joint_target_positions(q0, precision=True)
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

