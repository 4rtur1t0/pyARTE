#!/usr/bin/env python
# encoding: utf-8
"""
The script introduces you to the use of the classes:
    RotationMatrix

    ... and its representation on a plot

@Time: July 2022
"""
import numpy as np
from artelib.rotationmatrix import RotationMatrix, Rx, Ry, Rz
from artelib.vector import Vector

if __name__ == "__main__":
    # Diferentes formas de crear una matriz de Rotación
    # Una matriz de rotación identidad de 3x3
    R = RotationMatrix(3)
    R.plot('Identity 3x3')
    # Definiendo tres filas
    R = RotationMatrix([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    R.plot()
    # create different rotation matrices
    Ra = Rx(np.pi/4)
    Rb = Ry(np.pi/4)
    Rc = Rz(np.pi/4)

    Ra.plot(title='Rx pi/4')
    Rb.plot(title='Ry pi/4')
    Rc.plot(title='Rz pi/4')

    R = Ra*Rb*Rc
    R.plot(title='Three consecutive rotations of pi/4 along XYZ')

    # inverse and determinant
    R = R.inv()
    R = R.T()
    print('Inverse matrix (transposed, actually): ')
    print(R)
    print('Determinant: ', R.det())
    # Rotación de un vector
    u = Vector(np.array([1, 1, 1]))
    # Rotar un vector
    u = R*u.T()
    print(u)
    u.plot('Rotated vector')


