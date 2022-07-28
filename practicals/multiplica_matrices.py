import numpy as np


def multiplica_matrices():
    a = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    b = np.array([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12]])
    print('MATRICES:')
    print('A: ')
    print(a)
    print('B')
    print(b)
    print('PRODUCTO:')
    c = np.dot(a, b)
    print(c)
    print('DIMENSIONES: ')
    print(c.shape)


if __name__ == '__main__':
    multiplica_matrices()