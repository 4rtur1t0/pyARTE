#!/usr/bin/env python
# encoding: utf-8
"""
The script introduces you to the use of the classes:
    Vector

    ... and its representation on a plot

@Time: February 2023
"""
import numpy as np
from artelib.vector import Vector

if __name__ == "__main__":
    u = Vector(np.array([1, 1, 1]))
    v = Vector([1, 0, 1])
    print('u:', u)
    print('v:', v)

    # Traspuesto
    w = u.T()
    # suma y resta
    w = u + v
    w = u - v
    # producto escalar
    w = u*v.T()
    print('producto escalar: ', w)

    # producto vectorial
    w = u.cross(v)
    print('producto vectorial: ', w)

    u.plot()
    v.plot()
    w.plot()




