#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
import matplotlib.pyplot as plt


def plot_xy(x, y, title='UNTITLED'):
    plt.figure()
    x = np.array(x)
    y = np.array(y)
    x = x.flatten()
    y = y.flatten()
    plt.plot(x, y, linewidth=4, marker='.')
    # plt.xlabel('Distancia (m)')
    # plt.ylabel('Módulo (m/s)')
    plt.legend()
    plt.title(title)
    plt.show(block=True)


def f(x, y):
    return y*np.log(y)/x

def euler():
    xs = []
    ys = []
    h = 0.1
    x = 2.0
    y = np.e
    for i in range(11):
        xs.append(x)
        ys.append(y)
        y = y + h*f(x, y)
        x = x + h
    plot_xy(xs, ys, 'Ejemplo 1')


if __name__ == "__main__":
    euler()

