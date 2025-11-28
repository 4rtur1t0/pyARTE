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
from robots.twodofplanarrobot import TwoDOFRobot


def plot_path(q, qd, qdd, t):
    plt.figure()
    plt.plot(t, q[0, :], label='q1', linewidth=4)
    plt.plot(t, q[1, :], label='q2', linewidth=4)
    plt.plot(t, qd[0, :], label='qd1', linewidth=4)
    plt.plot(t, qd[1, :], label='qd2', linewidth=4)
    plt.plot(t, qdd[0, :], label='qdd1', linewidth=4)
    plt.plot(t, qdd[1, :], label='qdd2', linewidth=4)
    plt.legend()
    plt.show()



def plot_results(robot, q, qd, qdd, t):
    # plt.figure()
    # plt.plot(t, label='planned time')
    # plt.plot(robot.t, label='simulation time')
    # plt.show()
    # errors in position
    plt.figure()
    plt.plot(t, q[0, :], label='q1 reference', linewidth=4)
    plt.plot(t, q[1, :], label='q2 reference', linewidth=4)
    plt.plot(t, np.array(robot.q).T[0, :], label='q1 robot', linewidth=4)
    plt.plot(t, np.array(robot.q).T[1, :], label='q2 robot', linewidth=4)
    plt.legend()
    plt.show()

    # errors in speed
    plt.figure()
    plt.plot(t, qd[0, :], label='qd1 reference', linewidth=4)
    plt.plot(t, qd[1, :], label='qd2 reference', linewidth=4)
    plt.plot(t, np.array(robot.qd).T[0, :], label='qd1', linewidth=4)
    plt.plot(t, np.array(robot.qd).T[1, :], label='qd2', linewidth=4)
    plt.legend()
    plt.show()

    plt.figure()
    plt.title('Torques')
    plt.plot(t, np.array(robot.tau).T[0, :], label='Tau 1')
    plt.plot(t, np.array(robot.tau).T[1, :], label='Tau 2')
    plt.legend()
    plt.show()

def path_planning(q, qd, qdd, t, delta_t):
    """
    function[q_t, qd_t, qdd_t, time, k] = fifth_order(q, qd, qdd, t, delta_t)
    """
    # delta_t = 0.005
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
    # # append 5 more!!
    for i in range(10):
        q.append(q_t)
        qd.append(qd_t)
        qdd.append(qdd_t)
        t = np.append(t, ti + (i+1)*delta_t)
    q = np.array(q)
    qd = np.array(qd)
    qdd = np.array(qdd)
    return q, qd, qdd, t


def control(robot, q, qd, qdd, t):
    for i in range(len(t)):
        robot.save_state()
        # control_step_open(robot, q[i], qd[i], qdd[i], t[i])
        control_step_closed_loop(robot, q[:, i], qd[:, i], qdd[:, i], algorithm='acc_compensation')
    return


def control_step_open(robot, qref, qdref, qddref, t):
    tau = robot.inversedynamics(qref, qdref, qddref)
    robot.set_torques([tau])
    robot.wait(1)
    return


def control_step_closed_loop(robot, qref, qdref, qddref, algorithm):
    # get current state
    q, qd = robot.get_state()
    if algorithm == 'open_loop':
        # Usually incorrect, compute tau for the reference only
        tau = robot.inversedynamics(qref, qdref, qddref)
    elif algorithm == 'PD_precomputed':
        # precomputed torque with PD compensation (yes, qdd can be computed from reference)
        tau = robot.inversedynamics(q, qd, qddref)
        # Add a PD control action
        tau = tau + 0.3 * (qdref - qd) + 5.8 * (qref - q)
    elif algorithm == 'acc_compensation':
        # the compensation on qdd (adding a compensation based on the error)
        qdd = qddref + 3.5 * (qdref - qd) + 30.8 * (qref - q)
        tau = robot.inversedynamics(q, qd, qdd)
    # apply torques and continue
    robot.set_torques(tau)
    robot.wait(1)
    return


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = TwoDOFRobot(simulation=simulation)
    robot.I = [0.02146, 0.02146]
    robot.start()
    delta_t = simulation.get_simulation_time_step()
    print('delta_t is: ', delta_t)
    total_time = 2
    q1, qd1, qdd1, t = path_planning([0, np.pi/2], [0, 0], [0, 0], [0, total_time], delta_t)
    q2, qd2, qdd2, t = path_planning([0, np.pi/8], [0, 0], [0, 0], [0, total_time], delta_t)
    q = np.vstack((q1, q2))
    qd = np.vstack((qd1, qd2))
    qdd = np.vstack((qdd1, qdd2))
    # plot_path(q, qd, qdd, t)
    # perform control
    control(robot, q, qd, qdd, t)
    # plot error
    plot_results(robot, q, qd, qdd, t)

    simulation.stop()