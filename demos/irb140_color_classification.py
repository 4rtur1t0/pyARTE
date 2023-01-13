#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

Use the camera on the robot arm to detect a color using a simple image processing.
The mean color of the image is returned and compared to pure colors.
A simple classification in RGB is then used to place the pieces in three main destinations (without any order)

Please: beware that increasing the image resolution may lead to a slow execution.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from sceneconfig.scene_configs_irb140 import init_simulation_ABBIRB140


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

def find_color(robot):
    """
    Places the camera on top of the piece.
    Saves the image for inspection.
    Computes the image to get the mean RGB value and the color name (red, green, blue).
    """
    # position and orientation so that the camera sees the piece
    target_positions = [[0.6, 0.33, 0.5],
                        [0.6, 0.33, 0.4]]
    target_orientation = [0, np.pi, 0]
    q0 = np.array([0, 0, 0, 0, 0, 0])
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=Euler(target_orientation), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=Euler(target_orientation), q0=q1)
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    image, resolution = robot.get_image()
    # saves the image if needed
    # robot.save_image(image=image, resolution=resolution, filename='image.png')
    # get the mean color of the image captured
    mean_color = robot.get_mean_color(image, resolution)
    # define perfect colors as R, G, B. normalized
    colors = np.array([[1, 0, 0], [0., 1, 0], [0, 0, 1]])
    color_names = ['R', 'G', 'B']
    d = np.linalg.norm(colors-mean_color, axis=1)
    color_index = np.argmin(d)
    # return the closest color found
    color = color_names[color_index]
    print('Piece is: ', color)
    return color


def pick(robot):
    """
    Picks the piece from the conveyor belt.
    """
    target_positions = [[0.6, 0.267, 0.45],  # approximation
                        [0.6, 0.267, 0.39]] # pick
    target_orientations = [[0, np.pi, 0],
                           [0, np.pi, 0]]
    q0 = np.array([0, 0, 0, 0, 0, 0])
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=Euler(target_orientations[0]), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=Euler(target_orientations[1]), q0=q0)

    robot.open_gripper(precision=True)
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    robot.close_gripper(precision=True)
    robot.set_joint_target_positions(q1, precision=True)


def place(robot, i, color):
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
    robot.open_gripper(precision=True)
    robot.set_joint_target_positions(q3, precision=True)


def pick_and_place():
    robot, conveyor_sensor = init_simulation_ABBIRB140()
    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.set_joint_target_positions(q0, precision=True)
    n_pieces = 48
    for i in range(n_pieces):
        while True:
            if conveyor_sensor.is_activated():
                break
            robot.wait()

        robot.set_joint_target_positions(q0, precision=True)
        color = find_color(robot)
        pick(robot)
        robot.set_joint_target_positions(q0, precision=True)
        place(robot, i, color)

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()


if __name__ == "__main__":
    pick_and_place()

