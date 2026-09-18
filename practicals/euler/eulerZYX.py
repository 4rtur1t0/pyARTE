import numpy as np
from artelib.euler import Euler
# from artelib import rotationmatrix
from artelib.rotationmatrix import Rx, Ry, Rz
from artelib.tools import normalize_angle

class EulerZYXm(Euler):

    # def __init__(self, abg):
    #     if isinstance(abg, list):
    #         self.abg = np.array(abg)
    #     elif isinstance(abg, np.ndarray):
    #         self.abg = abg
    #     elif isinstance(abg, Euler):
    #         self.abg = abg.abg

    def R(self):
        return self.euler2rot()

    # def Q(self):
    #     return quaternion.Quaternion(euler2q(self.abg))

    def __str__(self):
        return str(self.abg)

    def euler2rot(self):
        Ra = Rz(self.abg[0])
        Rb = Ry(self.abg[1])
        Rc = Rx(self.abg[2])
        R = Ra * Rb * Rc
        return R

    def rot2euler(self, R):
        """
        Conversion from the rotation matrix R to Euler angles.
        The XYZ convention in mobile axes is assumed.
        """
        th = np.abs(np.abs(R[2, 0])-1.0)
        # R[0, 2] = min(R[0, 2], 1)
        # R[0, 2] = max(R[0, 2], -1)

        # caso no degenerado
        if th > 0.0001:
            beta1 = np.arcsin(-R[2, 0])
            beta2 = np.pi - beta1
            s1 = np.sign(np.cos(beta1))
            s2 = np.sign(np.cos(beta2))
            alpha1 = np.arctan2(s1*R[1, 0], s1*R[0, 0])
            alpha2 = np.arctan2(s2 * R[1, 0], s2 * R[0, 0])
            gamma1 = np.arctan2(s1*R[2, 1], s1*R[2, 2])
            gamma2 = np.arctan2(s2*R[2, 1], s2*R[2, 2])
        # degenerate case
        else:
            print('CAUTION: rot2euler detected a degenerate solution when computing the Euler angles.')
            alpha1 = 0
            alpha2 = np.pi
            beta1 = np.arcsin(-R[2, 0])
            if beta1 > 0:
                beta2 = np.pi/2
                gamma1 = -np.arctan2(R[1, 2], R[0, 2])
                gamma2 = -np.arctan2(R[1, 0], R[1, 1])+alpha2
            else:
                beta2 = -np.pi/2
                gamma1 = np.arctan2(-R[1, 2], -R[0, 2])
                gamma2 = np.arctan2(-R[1, 2], -R[0, 2])-alpha2
        # finally normalize to +-pi
        e1 = normalize_angle([alpha1, beta1, gamma1])
        e2 = normalize_angle([alpha2, beta2, gamma2])
        return EulerZYXm(e1), EulerZYXm(e2)


if __name__ == '__main__':
    print('Creamos un objeto EulerXYZm')
    e = EulerZYXm([np.pi/4, -np.pi/2, np.pi/4])
    print('Matriz de rotación:')
    R = e.R()
    R.print()
    print('Soluciones para Euler:')
    [e1, e2] = e.rot2euler(R)
    print(e1)
    print(e2)
    print('Comprobamos las soluciones:')
    R1 = e1.R()
    R1.print()
    R2 = e2.R()
    R2.print()


