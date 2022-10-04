import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.tools import buildT


if __name__ == '__main__':
    orientation1 = [np.pi/2, np.pi/4, np.pi/2]
    orientation2 = [np.pi / 4, np.pi / 4, np.pi / 2]
    position1 = [0.5, 0.7, 0.8]
    position2 = [0.5, 0.7, 0.8]
    e1 = Euler(orientation1)
    e2 = Euler(orientation2)
    print('Dos conjuntos de ángulos de Euler')
    print(e1.abg)
    print(e2.abg)
    # Conversión de Euler a Matriz de rotacion
    R1 = e1.R()
    R2 = e2.R()
    # combinacion de rotaciones
    R = R1*R2
    print('R1, R2 y R:')
    print(R1)
    print(R2)
    print(R)
    R.plot()

    print('Ángulos de Euler de R:')
    # conversión de R a ángulos de Euler
    [e1, e2] = R.euler()
    print(e1.abg, e2.abg)

    # Creamos una matriz Homogenea
    T1 = buildT(position1, R1)
    T1 = HomogeneousMatrix(T1)
    T2 = buildT(position2, R2)
    T2 = HomogeneousMatrix(T2)

    # Composición de transformaciones
    T = T1*T2
    print(T)
    print(T.R())
    print(T.pos())
