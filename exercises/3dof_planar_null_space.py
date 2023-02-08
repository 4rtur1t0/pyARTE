"""
EXERCISE: Compute the projector P to the null space of a 3 dof planar robot.
Consider that the robot is working in the workspace m=2 with direct kinematics:
(vx, vy)^T=J·dq and is at q=[pi/2 pi/2 pi/2]
"""
import numpy as np


if __name__ == '__main__':

    q1 = np.pi/2
    q2 = np.pi/2
    q3 = np.pi/2
    qd0 = np.array([1, 1, 1])
    s123 = np.sin(q1+q2+q3)
    s12 = np.sin(q1+q2)
    s1 = np.sin(q1)
    c123 = np.cos(q1+q2+q3)
    c12 = np.cos(q1+q2)
    c1 = np.cos(q1)
    J = np.array([[-s1-s12-s123, -s12-s123, -s123],
                  [c1+c12+c123, c12+c123, c123]])
    print('J')
    print(J)
    jj = np.linalg.inv(np.dot(J, J.T))
    P = np.eye(3)-np.dot(J.T, np.dot(jj, J))
    print(P)
    qd = np.dot(P, qd0.T)

    print('qd at null space')
    print(qd)
    print('end effector speed')
    v = np.dot(J, qd.T)
    print(v.T)