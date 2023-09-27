#!/usr/bin/env python
# encoding: utf-8
"""
The demo plots de values of a secondary objective function for some values of q

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.plottools import plot_xy
from artelib.tools import w_lateral, diff_w_lateral


def plot_secondary_objectives():
    """
    Plot lateral function
    """
    qmin = -np.pi/2
    qmax = np.pi/2

    qs = np.linspace(qmin, qmax, 500)
    x = []
    y = []
    for q in qs:
        w = w_lateral([q], [qmin], [qmax])
        x.append(q)
        y.append(w)
    plot_xy(x, y, title='Lateral function')

    x = []
    y = []
    for q in qs:
        dw = diff_w_lateral([q], [qmin], [qmax])
        x.append(q)
        y.append(dw)
    plot_xy(x, y, title='Lateral function (derivative)')


if __name__ == "__main__":
    plot_secondary_objectives()

