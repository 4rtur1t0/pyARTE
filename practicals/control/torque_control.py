"""
Simple torque based control 1

1 Plan a path (q1 to q2)
         position,
         speed
         acceleration.
2 At each time step, compute the inverse dynamics.

3 Apply torque along with a PID correction based on the error in position, speed and acceleration.

"""
import numpy as np
from robots.onedofrobot import OneDOFRobot
from robots.simulation import Simulation
import matplotlib.pyplot as plt


def plot_path(q, qd, qdd, t):
    plt.figure()
    plt.plot(t, q, label='q', linewidth=4)
    plt.plot(t, qd, label='qd', linewidth=4)
    plt.plot(t, qdd, label='qdd', linewidth=4)
    plt.show()
    plt.legend()


def path_planning(q, qd, qdd, t):
    """
    function[q_t, qd_t, qdd_t, time, k] = fifth_order(q, qd, qdd, t, delta_t)
    """
    delta_t = 0.05
    A = np.array([[1, t[0], t[0]**2, t[0]**3, t[0]**4, t[0] ** 5],
                  [1, t[1], t[1]**2, t[1]**3, t[1]**4, t[1]**5],
                  [0,    1,  2*t[0], 3*t[0]**2, 4*t[0]**3, 5*t[0]**4],
                  [0,    1,  2*t[1], 3*t[1]**2, 4*t[1]**3, 5*t[1]**4],
                  [0,    0,   2,   6*t[0], 12*t[0]**2, 20 * t[0] ** 3],
                  [0,    0,   2, 6 * t[1], 12 * t[1] ** 2, 20 * t[1] ** 3]])
    
    # A * k = b
    b = np.array([q[0], q[1], qd[0], qd[1], qdd[0], qdd[1]])

    #k = [k[1] k[2] k[3] k[4] k[5] k[6]]
    k = np.dot(np.linalg.inv(A), b.T)
    n = int(np.floor((t[1]-t[0])/delta_t))
    t = np.linspace(t[0], t[1], n+1)
    q = []
    qd = []
    qdd = []
    # ecuacion: q(t) = k1 + k2 * t + k3 * t ^ 2 + k4 * t ^ 3 + k5 * t ^ 4 + k6 * t ^ 5
    # % qd(t) = k2 + 2 * k3 * t + 3 * k4 * t ^ 2 + 4 * k5 * t ^ 3 + 5 * k6 * t ^ 4
    # % qdd(t) = 2 * k3 + 6 * k4 * t + 12 * k5 * t ^ 2 + 20 * k6 * t ^ 3
    # time = t(1):delta_t: t(2)
    for ti in t:    
        q_t = k[0] + k[1] * ti + k[2] * ti ** 2 + k[3] * ti ** 3 + k[4] * ti ** 4 + k[5] * ti ** 5
        qd_t = k[1] + 2 * k[2] * ti + 3 * k[3] * ti ** 2 + 4 * k[4] * ti ** 3 + 5 * k[5] * ti ** 4
        qdd_t = 2 * k[2] + 6 * k[3] * ti + 12 * k[4] * ti ** 2 + 20 * k[5] * ti ** 3
        q.append(q_t)
        qd.append(qd_t)
        qdd.append(qdd_t)
    q = np.array(q)
    qd = np.array(qd)
    qdd = np.array(qdd)
    return q, qd, qdd, t


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = OneDOFRobot(simulation=simulation)
    robot.start()
    g = 9.81
    m = 1
    L = 1
    I = 8

    q, qd, qdd, t = path_planning([0, np.pi/2], [0, 0], [0, 0], [0, 2])

    qi, qdi, qddi = robot.get_state()
    tau = robot.inversedynamics()

    robot.set_torques([tau])
    # simulation.wait_time(50)



    simulation.stop()