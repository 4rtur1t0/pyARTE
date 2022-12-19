import numpy as np


def rot(alpha, axis):
    if axis == 'x':
        return np.array([[1, 0, 0],
                         [0, np.cos(alpha), -np.sin(alpha)],
                         [0, np.sin(alpha), np.cos(alpha)]])
    elif axis == 'y':
        return np.array([[np.cos(alpha), 0, np.sin(alpha)],
                         [0, 1, 0],
                         [-np.sin(alpha), 0, np.cos(alpha)]])
    else:
        return np.array([[np.cos(alpha), -np.sin(alpha), 0],
                         [np.sin(alpha), np.cos(alpha), 0],
                         [0, 0, 1]])


if __name__ == '__main__':
    Rx = rot(np.pi/2, 'x')
    Ry = rot(-np.pi/2, 'y')
    Rz = rot(np.pi, 'z')
    print(Rx)
    print(Ry)
    print(Rz)
