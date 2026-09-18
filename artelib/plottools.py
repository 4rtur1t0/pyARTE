#!/usr/bin/env python
# encoding: utf-8
"""
A number of miscellaneous tools
@Authors: Arturo Gil
@Time: April 2021

"""
import matplotlib.pyplot as plt
import numpy as np


def plot(x, title='Untitled', block=True):
    plt.figure()
    x = np.array(x)
    plt.plot(x)
    plt.title(title)
    plt.show(block=block)


def plot_vars(q_path, title='UNTITLED'):
    plt.figure()
    q_path = np.array(q_path)
    sh = q_path.shape
    for i in range(0, sh[1]):
        plt.plot(q_path[:, i], label='q' + str(i + 1), linewidth=4)
    plt.legend()
    plt.title(title)
    plt.show(block=True)


def plot_xy(x, y, title='UNTITLED'):
    plt.figure()
    x = np.array(x)
    y = np.array(y)
    x = x.flatten()
    y = y.flatten()
    plt.plot(x, y, linewidth=4, marker='.')
    # plt.xlabel('qi')
    # plt.ylabel('dw')
    plt.legend()
    plt.title(title)
    plt.show(block=True)


def plot3d(x, y, z, title='3D'):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.xlim([0, 0.5])
    plt.title(title)
    plt.show(block=True)
