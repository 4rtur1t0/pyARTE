import numpy as np


def normalize(eul):
    e = []
    for i in range(len(eul)):
        e.append(np.arctan2(np.sin(eul[i]), np.cos(eul[i])))
    return e


def test_solution(R, Rtest):
    Rs = np.abs(R - Rtest)
    s = Rs.sum()
    assert(s < 0.001)


def rot2euler(R):
    th = np.abs(np.abs(R[0, 2])-1.0)
    # caso no degenerado
    if th > 0.0001:
        beta1 = np.arcsin(R[0, 2])
        beta2 = np.pi - beta1
        alpha1 = np.arctan2(-R[1][2]/np.cos(beta1), R[2][2]/np.cos(beta1))
        gamma1 = np.arctan2(-R[0][1]/np.cos(beta1), R[0][0]/np.cos(beta1))
        alpha2 = np.arctan2(-R[1][2]/np.cos(beta2), R[2][2]/np.cos(beta2))
        gamma2 = np.arctan2(-R[0][1]/np.cos(beta2), R[0][0]/np.cos(beta2))
    else:
        # EJERCICIO, TERMINE DE CODIFICAR LA FUNCIÓN [...]

    # finally normalize to +-pi
    e1 = normalize([alpha1, beta1, gamma1])
    e2 = normalize([alpha2, beta2, gamma2])
    return e1, e2


def euler2rot(abg, convention='xyz'):
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
    R = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    e = rot2euler(R)
    print('ANGULOS DE EULER ')
    print(e)
    test_solution(R, euler2rot(e[0]))
    test_solution(R, euler2rot(e[1]))

    R = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    e = rot2euler(R)
    print('ANGULOS DE EULER ')
    print(e)
    test_solution(R, euler2rot(e[0]))
    test_solution(R, euler2rot(e[1]))

    R = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    e = rot2euler(R)
    print('ANGULOS DE EULER ')
    print(e)
    test_solution(R, euler2rot(e[0]))
    test_solution(R, euler2rot(e[1]))

    R = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]])
    e = rot2euler(R)
    print('ANGULOS DE EULER ')
    print(e)
    test_solution(R, euler2rot(e[0]))
    test_solution(R, euler2rot(e[1]))

    # ángulos arbitrarios
    R = euler2rot([-np.pi/4, np.pi/4, np.pi/4])
    e = rot2euler(R)
    print('ANGULOS DE EULER ')
    print(e)
    test_solution(R, euler2rot(e[0]))
    test_solution(R, euler2rot(e[1]))

