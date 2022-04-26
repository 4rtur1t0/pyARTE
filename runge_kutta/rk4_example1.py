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


def f(t, y):
    fty = (5*t**2-y)/np.exp(t+y)
    return fty


def rk4():
    xs = []
    ys = []
    h = 0.1
    t = 0.0
    y = 1
    for i in range(11):
        xs.append(t)
        ys.append(y)
        f1 = h*f(t, y)
        f2 = h*f(t+h/2, y+f1/2)
        f3 = h*f(t+h/2, y+f2/2)
        f4 = h*f(t+h, y + f3)
        y = y + (1 / 6) * (f1 + 2 * f2 + 2 * f3 + f4)
        t = t + h

    plot_xy(xs, ys, 'Ejemplo 1')


if __name__ == "__main__":
    rk4()

