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
    """
    Debe devolver la trayectoria de una letra.
    """
    return #letter_traj, w, h


def generate_text_trajectory(text):
    """
    Genera la trayectoria correspondiente al texto.
    No es necesario escalar las letras.
    Debe llamar a la función generate_letter_trajectory para cada letra
    """
    robot_trajectory = []
    return #robot_trajectory


def view_hershey_data():
    # ejecuta estas líneas para ver las letras por separado
    text = 'hola'
    for character in text:
        x, y, w, h = get_trajectory_for_letter(character=character)
        plot_letter(x, y)
    # A COMPLETAR: la función generate_text_trajectory
    robot_trajectory = generate_text_trajectory(text='abcde, ABCDE!')

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    for letter_trajectory in robot_trajectory:
        ax.scatter(letter_trajectory[:, 0], letter_trajectory[:, 1], letter_trajectory[:, 2])
        plt.show()
        # plot_points(letter_trajectory)



if __name__ == "__main__":
    view_hershey_data()


