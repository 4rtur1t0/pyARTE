import numpy as np


def euler2rot(abg, convention):
    if convention == 'xyz':
        Ra = rot(abg[0], 'x')
        Rb = rot(abg[1], 'y')
        Rc = rot(abg[2], 'z')
    elif convention == 'zxz':
        Ra = rot(abg[0], 'z')
        Rb = rot(abg[1], 'x')
        Rc = rot(abg[2], 'z')
    elif convention == 'xzx':
        Ra = rot(abg[0], 'x')
        Rb = rot(abg[1], 'z')
        Rc = rot(abg[2], 'x')
    else:
        print('UNKNOWN EULER ANGLE CONVENTION!')
        raise Exception
    R = np.dot(Ra, np.dot(Rb, Rc))
    return R


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
    Rxyz = euler2rot([np.pi/2, 0, np.pi/2], 'xyz')
    Rzxz = euler2rot([np.pi/2, 0, np.pi/2], 'zxz')
    Rxzx = euler2rot([np.pi/2, 0, np.pi/2], 'xzx')

    print(Rxyz)
    print(Rzxz)
    print(Rxzx)


