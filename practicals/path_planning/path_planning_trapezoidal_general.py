#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script computes the inverse kinematic of the IRB140 robot and sends joint values to Coppelia to view
the results.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
import matplotlib.pyplot as plt


def path_trapezoidal(qA, qB, qdA, T, endpoint=False):
    delta_time = 0.05
    if endpoint:
        Ta = 0.2
        Td = 0.2
        qdB = 0
        # Waypoint CaseA
        Tcte = T - Ta - Td
        if Tcte <= 0:
            Tcte = 0
        qdcte = (qB - qA - 0.5 * (qdA * Ta + qdB * Td)) / (Tcte + 0.5 * (Ta + Td))
    else:
        Ta = 0.2
        Td = 0.2
        # Waypoint CaseA
        Tcte = T - Ta - Td
        if Tcte <= 0:
            Tcte = 0
        qdcte = (qB - qA - 0.5*qdA*Ta)/(Tcte + Td + 0.5*Ta)
        qdB = qdcte

    # the two waypoints in the trapezoidal profile
    q1 = qA + qdA * Ta + 0.5 * (qdcte - qdA) * Ta
    q2 = q1 + qdcte * Tcte
    # matrix of restrictions to match qa, qdA, qb, qdB and qdcte
    A = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [1, Ta, Ta**2, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 1, Ta, Ta**2, 0, 0, 0],
                  [0, 0, 0, 0, 1, 2*Ta, 0, 0, 0],
                  [0, 0, 0, 1, (Ta+Tcte), (Ta+Tcte)**2, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, (Ta+Tcte), (Ta+Tcte)**2],
                  [0, 0, 0, 0, 0, 0, 0, 1, 2*(Ta+Tcte)],
                  [0, 0, 0, 0, 0, 0, 1, T, T**2]])
    print(A)
    Q = np.array([qA, qdA, q1,
                  q1, qdcte, q2,
                  q2, qdcte, qB])
    k = np.dot(np.linalg.inv(A), Q)
    n_samples = int(np.round(T/delta_time))
    t = np.linspace(0, T, n_samples)
    qt = []
    qdt = []
    for ti in t:
        if ti <= Ta:
            qt.append(k[0] + k[1]*ti + k[2]*ti**2)
            qdt.append(k[1] + 2*k[2]*ti)
        elif Ta < ti <= Ta+Tcte:
            qt.append(k[3] + k[4] * ti + k[5] * ti ** 2)
            qdt.append(k[4] + 2 * k[5] * ti)
        else:
            qt.append(k[6] + k[7] * ti + k[8] * ti ** 2)
            qdt.append(k[7] + 2 * k[8] * ti)
    return t, np.array(qt), np.array(qdt)


def time_path(qA, qB, qdA, qdmax, endpoint=False):
    """
    Computes the time needed for a trapezoidal speed profile.
    The joint must move form joint position qA to joint position qB
    The starting speed at position qA is qdA.
    The joint is assumed to move at a max speed of qdmax.
    In case of an endpoint
    """
    delta_time = 0.05
    # and end point with three segments
    if endpoint:
        Ta = 0.2
        Td = 0.2
        qdB = 0
    # Waypoint Case
    else:
        Ta = 0.2
        Td = 0.2
        qdB = qdmax
    Tcte = (qB - qA - 0.5 * (qdA + qdmax)*Ta - 0.5 * (qdB + qdmax)*Td) / qdmax
    # if the time at constant speed is negative, then clip to zero
    if Tcte <= 0:
        Tcte = 0
    # compute total time
    T = Tcte + Ta + Td
    # round to next sample time
    n = np.floor(T/delta_time) + 1
    T = delta_time*n
    return T


def CASEA1(endpoint):
    qdmax = 8 # rad/s
    qA = 0
    qB = -0.9
    qdA = 0.5

    ttotal = time_path(qA, qB, qdA, qdmax, endpoint=endpoint)
    print('COMPUTED TIME FOR JOINT I: ', ttotal)
    t, qt, qdt = path_trapezoidal(qA, qB, qdA, ttotal, endpoint=endpoint)
    plt.plot(t, qt)
    plt.show()
    plt.plot(t, qdt)
    plt.show()


def CASEA2(endpoint):
    qdmax = 0.2  # rad/s
    qA = 0.5
    qdA = -0.8
    qB = 1.0
    T = time_path(qA, qB, qdA, qdmax, endpoint=endpoint)
    t, qt, qdt = path_trapezoidal(qA, qB, qdA, T, endpoint=endpoint)
    plt.plot(t, qt)
    plt.show()
    plt.plot(t, qdt)
    plt.show()


def CASEB1(endpoint):
    # endpoint, triangle case
    qdmax = 0.1  # rad/s
    qA = 0.5
    qdA = 0.0
    qB = 0.5
    T = time_path(qA, qB, qdA, qdmax, endpoint=endpoint)
    t, qt, qdt = path_trapezoidal(qA, qB, qdA, T, endpoint=endpoint)
    plt.plot(t, qt)
    plt.show()
    plt.plot(t, qdt)
    plt.show()

def CASEB2(endpoint):
    qdmax = 2.5  # rad/s
    qA = 0.5
    qdA = 2.8
    qB = 0.55
    T = time_path(qA, qB, qdA, qdmax, endpoint=endpoint)
    t, qt, qdt = path_trapezoidal(qA, qB, qdA, T, endpoint=endpoint)
    plt.plot(t, qt)
    plt.show()
    plt.plot(t, qdt)
    plt.show()





if __name__ == "__main__":
    endpoint = True
    CASEA1(endpoint=endpoint)
    #test_CASEA2(endpoint=endpoint)
    # test_CASEB1(endpoint=endpoint)
    # test_CASEB2(endpoint=endpoint)


