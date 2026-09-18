#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_project_soldering.ttt scene before running this script.

The code beneath must be completed by the student in order to engrave/solder letters on
 a metal plate.

@Authors: Arturo Gil
@Time: November 2025
"""
import numpy as np
# from pexpect.pxssh import pxssh
#
# from artelib.euler import Euler
# from artelib.path_planning import compute_3D_coordinates
# from artelib.vector import Vector
# from artelib.homogeneousmatrix import HomogeneousMatrix
# from robots.abbirb140 import RobotABBIRB140
# from robots.grippers import SuctionPad
# from robots.objects import get_object_transform, ReferenceFrame
# from robots.proxsensor import ProxSensor
# from robots.simulation import Simulation
# from robots.camera import Camera
import matplotlib.pyplot as plt
from hershey import hershey_dict


def get_trajectory_for_letter(character):
    # get coordinates
    her = hershey_dict[ord(character)]
    strokes = her['stroke']
    x = strokes[0]
    y = strokes[1]
    width = her['width']
    height = her['top']
    return x, y, width, height


def plot_letter(x, y):
    plt.plot(np.array(x), np.array(y))
    plt.show()


def plot_points(text_points):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(text_points[:,0], text_points[:,1], text_points[:,2])
    plt.show()


def generate_letter_trajectory(character):
    letter_traj = []
    start_point_height = 0.02
    x, y, w, h = get_trajectory_for_letter(character=character)
    letter_traj.append(np.array([x[0], y[0], start_point_height]))
    letter_traj.append(np.array([x[0], y[0], 0]))
    for i in range(len(x)-1):
        if x[i+1] is None:
            # salida dentro de la letra
            letter_traj.append(np.array([x[i], y[i], start_point_height]))
            continue
        else:
            letter_traj.append(np.array([x[i+1], y[i+1], 0]))
    # salida final de la letra
    letter_traj.append(np.array([x[i+1], y[i+1], start_point_height]))
    letter_traj = np.array(letter_traj)
    # plot_letter(letter_traj[:,0], letter_traj[:,1])
    # plot_points(letter_traj)
    return letter_traj, w, h


def generate_text_trajectory(text):
    linenum = 0
    robot_trajectory = []
    #current letter position
    current_letter_position = np.array([0, 0, 0])
    for character in text:
        if character == ' ':
            current_letter_position = current_letter_position + np.array([0.5, 0, 0])
            continue
        elif character == '\n':
            linenum += 1
            # carriage return!
            current_letter_position = np.array([0, -linenum, 0])
            continue
        traj, w, h = generate_letter_trajectory(character=character)
        robot_trajectory.append(traj+current_letter_position)
        current_letter_position = current_letter_position + np.array([w, 0, 0])
    # robot_trajectory = np.array(robot_trajectory)
    return robot_trajectory


def view_hershey_data():
    # text = 'hola'
    # for character in text:
    #     x, y, w, h = get_trajectory_for_letter(character=character)
    #     plot_letter(x, y)

    robot_trajectory = generate_text_trajectory(text='abcde, ABCDE!')
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    for letter_trajectory in robot_trajectory:
        ax.scatter(letter_trajectory[:, 0], letter_trajectory[:, 1], letter_trajectory[:, 2])
        plt.show()
        # plot_points(letter_trajectory)



if __name__ == "__main__":
    view_hershey_data()


