#!/usr/bin/env python
# encoding: utf-8
"""
Simple trajectory planning class.
Implements a trapezoidal profile

@Authors: Arturo Gil
@Time: Febrer de 2026

"""
import numpy as np
import matplotlib.pyplot as plt


class TrajectoryPlanner():
    """
        Used in the MoveAbsJ, MoveJ and MoveL functions.
    """
    def __init__(self, joint_parameters, delta_time, settle_time):
        self.v_max = joint_parameters.max_joint_speeds
        self.a_max = joint_parameters.max_joint_accelerations
        self.delta_time = delta_time
        self.settle_time = settle_time
        self.DOF = joint_parameters.DOF

    def trapezoidal_coordinated(self, s0, sf, v_max, a_max, precision=False, plot=False):
        """
            qd_factor modifies, up to one, the max speed of the joints. I. e. a qd_factor of 0.5 plans
            the movement using half of its maximum speed. A value of qd_factor of 1.0 matches exactly
            the maximum joint speeds. qd_factor must be > 0, but it is not limited to 1.0, so the
            user may experience using values of qd_factor greater than 1.0.
            qdd_factor has the same behaviour, but it applies to the acceleration.
        """
        ttotal = []
        st = []
        sdt = []
        sddt = []
        # Plan a path in joint or task space
        for i in range(len(s0)):
            # find the time that each joint needs, considering that it may work at max speed and
            # acceleration
            [ttotal_i, traj_type] = self.compute_time_sign(s0[i], sf[i], v_max[i], a_max[i])
            ttotal.append(ttotal_i)
        # find  the  max  time:  the total coordinated time is chosen as the time corresponding to the slowest joint
        t_coord = np.max(ttotal)
        # aim for an exact match above
        n_samples = np.ceil(t_coord/self.delta_time)
        n_samples = int(n_samples)
        t_coord = n_samples*self.delta_time
        # now, plan   all   the   joints    using    the  coordinated  time for a isochronous movement.
        # cauition, if precision is set, a number of samples are added to the output, so that
        # the controller may have enough time to get a full control of the joints
        for i in range(len(s0)):
            [st_, sdt_, sddt_, time] = self.caseB(s0[i], sf[i], t_coord, a_max[i],
                                                  precision)
            # append to the total output
            st.append(st_)
            sdt.append(sdt_)
            sddt.append(sddt_)
        # convert to a numpy array
        st = np.array(st)
        sdt = np.array(sdt)
        sddt = np.array(sddt)
        if plot:
            self.plot_planned_trajectories(st, sdt, sddt, time, len(s0))
        return st, sdt, sddt, time

    def trapezoidal_coordinated_approx(self, s0, sf, v_max, a_max, precision=False, plot=False):
        """
            A trapezoidal coordination  considering speeds and accelerations, followed by
            a real quintic planning for smoother operation. The reason for this is to
            easy the control of the robot, which, in the absence of friction, is complex.
            qd_factor modifies, up to one, the max speed of the joints. I. e. a qd_factor of 0.5 plans
            the movement using half of its maximum speed. A value of qd_factor of 1.0 matches exactly
            the maximum joint speeds. qd_factor must be > 0, but it is not limited to 1.0, so the
            user may experience using values of qd_factor greater than 1.0.
            qdd_factor has the same behaviour, but it applies to the acceleration.
        """
        ttotal = []
        # Plan a path in joint or task space
        for i in range(len(s0)):
            # find the time that each joint needs, considering that it may work at max speed and
            # acceleration
            [ttotal_i, traj_type] = self.compute_time_sign(s0[i], sf[i], v_max[i], a_max[i])
            ttotal.append(ttotal_i)
        # find  the  max  time:  the total coordinated time is chosen as the time corresponding to the slowest joint
        t_coord = np.max(ttotal)
        # aim for an exact match above
        n_samples = np.ceil(t_coord/self.delta_time)
        n_samples = int(n_samples)
        t_coord = n_samples*self.delta_time
        # now, plan   all   the   joints    using    the  coordinated  time for a isochronous movement.
        # cauition, if precision is set, a number of samples are added to the output, so that
        # the controller may have enough time to get a full control of the joints
        st, sdt, sddt, time = self.quintic_trajectory(q0=s0,
                                                      qf=sf,
                                                      qd0=np.zeros(self.DOF),
                                                      qdf=np.zeros(self.DOF),
                                                      qdd0=np.zeros(self.DOF),
                                                      qddf=np.zeros(self.DOF),
                                                      total_time=t_coord,
                                                      precision=precision)
        if plot:
            self.plot_planned_trajectories(st, sdt, sddt, time, len(s0))
        return st, sdt, sddt, time

    def compute_time_sign(self, q0, qf, omega, alpha):
        if omega == 0.0:
            print('ERROR: the max speed cannot be zero')
            return 0, 'unfeasible'
        if alpha == 0.0:
            print('ERROR: the max acceleration cannot be zero')
            return 0, 'unfeasible'

        # actual speeds and accelerations with sign
        omega = omega * np.sign(qf - q0)
        alpha = alpha * np.sign(qf - q0)
        [ttotal, traj_type] = self.compute_time(q0, qf, omega, alpha)
        # print('\n\nCASE A. ttotal: %f, traj_type: %s\n', ttotal, traj_type)
        return ttotal, traj_type

    def caseA(self, q0, qf, omega, alpha):
        # actual speeds and accelerations with sign
        omega = omega * np.sign(qf - q0)
        alpha = alpha * np.sign(qf - q0)
        [ttotal, traj_type] = self.compute_time(q0, qf, omega, alpha)
        [qt, qdt, qddt, time] = self.trapezoidal_profile(q0, qf, omega, alpha, ttotal, traj_type)
        # (q0, qdmax, qddmax, tcte, tacc, delta_time)
        # print('\n\nCASE A. ttotal: %f, traj_type: %s\n', ttotal, traj_type)
        return ttotal, traj_type, qt, qdt, qddt, time

    def caseB(self, q0, qf, ttotal, alpha, precision):
        # % actual acceleration with sign
        alpha = alpha * np.sign(qf - q0)
        [omega, traj_type] = self.compute_speed(q0, qf, ttotal, alpha)
        # caution, indicating a new speed omega
        [qt, qdt, qddt, time] = self.trapezoidal_profile(q0, qf, omega, alpha, ttotal, traj_type, precision)
        # print('\n\nCASE B. ttotal: %f, traj_type: %s\n', ttotal, traj_type)
        return qt, qdt, qddt, time

    def compute_time(self, q0, qf, omega, alpha):
        deltaq = qf - q0
        # standard trapezoidal case. This will be returned by default.
        traj_type = 'trapezoidal'
        # deal with the case in which no movement should be produced
        if abs(deltaq) == 0.0:
            ttotal = 0.0
            traj_type = 'nomovement'
            return ttotal, traj_type
        tacc = omega / alpha
        tcte = deltaq / omega - omega / alpha
        # triangular case
        if tcte <= 0.0:
            tcte = 0.0
            tacc = np.sqrt(deltaq / alpha)
            traj_type = 'triangular'
        ttotal = tcte + 2 * tacc
        return ttotal, traj_type

    # Given delta q and qddmax, computes two possible solutions for qdmax
    # The one within specs is chosen.
    def compute_speed(self, q0, qf, ttotal, alpha):
        deltaq = qf - q0
        # in this case, the deltaq = 0 is handled correctly by default, returning
        # a triangular trajectory that will be correctly generated
        # compute the total time
        a = 1.0
        b = -ttotal * alpha
        c = deltaq * alpha
        omega1 = (-b + np.sqrt((b**2) - (4 * a * c))) / (2 * a)
        omega2 = (-b - np.sqrt((b**2) - (4 * a * c))) / (2 * a)
        # this case is unfeasible.Given alpha
        # if np.imag(omega1) > 0:
        if np.isnan(omega1) or np.isnan(omega2):
            # print('Unfeasible')
            # print('The movement cannot be performed with this alpha')
            # print('Increase acceleration alpha')
            omega = 0
            traj_type = 'unfeasible'
            return omega, traj_type
        if omega1 == omega2:
            omega = omega1
            traj_type = 'triangular'
            return omega, traj_type
        omegas = [omega1, omega2]
        idx = np.argmin(np.abs(omegas))
        omega = omegas[idx]
        traj_type = 'trapezoidal'
        return omega, traj_type

    def trapezoidal_profile(self, q0, qf, omega, alpha, ttotal, traj_type, precision):
        qd0 = 0
        if traj_type == 'triangular':
            tcte = 0.0
            tacc = ttotal/2.0
            deltaq = (qf-q0)
            # %alpha=(qf-q0)/tacc^2;
            omega = deltaq/tacc
            # print('Triangular profile')
            # recompute the max speed of the profile
            q1 = q0 + alpha*tacc**2/2
            q2 = q1 + tcte*omega
            # % build the global time vector
            # time = 0:delta_time:(tacc+tacc)
            # time = np.arange(0.0, tacc + tacc + self.delta_time, self.delta_time)
            time = np.arange(0.0, ttotal + self.delta_time, self.delta_time)
            timeA = time[(time >= 0.0) & (time <= tacc)]
            # timeB = []
            timeC = time[time > (tacc+tcte)]-tacc
            [qtA, qdtA, qddtA] = self.acceleration([q0, qd0, alpha], timeA)
            qtB = []
            qdtB = []
            qddtB = []
            [qtC, qdtC, qddtC] = self.acceleration([q2, omega, -alpha], timeC)
        else:
            # print('Trapezoidal profile')
            tacc = omega/alpha
            tcte = ttotal - 2.0*tacc
            q1 = q0 + alpha*tacc**2/2.0
            q2 = q1 + tcte*omega
            #  generate a sampled time. Then filter each local time and
            #  refer it to zero.
            # time = 0:delta_time: (tacc + tacc + tcte)
            # increase by delta_time, so that tcte is always reached
            # time = np.arange(0.0, tacc + tacc + tcte + self.delta_time, self.delta_time)
            time = np.arange(0.0, ttotal + self.delta_time, self.delta_time)
            timeA = time[(time >= 0.0) & (time <= tacc)]
            timeB = time[(time > tacc) & (time <= (tacc+tcte))]-tacc
            timeC = time[time > (tacc+tcte)]-tacc-tcte
            # compute profiles
            [qtA, qdtA, qddtA] = self.acceleration([q0, qd0, alpha], timeA)
            [qtB, qdtB, qddtB] = self.constant_speed([q1, omega], timeB)
            [qtC, qdtC, qddtC] = self.acceleration([q2, omega, -alpha], timeC)
        # build the global q(t), qd(t) and qdd(t)
        qt = np.hstack((qtA, qtB, qtC))
        qdt = np.hstack((qdtA, qdtB, qdtC))
        qddt = np.hstack((qddtA, qddtB, qddtC))

        if precision:
            n_settle = int(self.settle_time / self.delta_time)
        else:
            n_settle = 0
        # append the last values!!
        t_last = time[-1]
        for i in range(n_settle):
            qt = np.append(qt, qt[-1])
            qdt = np.append(qdt, qdt[-1])
            qddt = np.append(qddt, qddt[-1])
            time = np.append(time, t_last + (i + 1) * self.delta_time)
        return qt, qdt, qddt, time

    def acceleration(self, b, time):
        """
        Compute the function regarding the acceleration phase
        """
        # second order polynomial
        q_t = b[0] + b[1]*time + b[2]*np.square(time)/2
        # speed
        qd_t = b[1] + b[2]*time
        # return a constant accel
        qdd_t = b[2]*np.ones(len(time))
        return q_t, qd_t, qdd_t

    def constant_speed(self, b, time):
        # the first order equation
        q_t = b[0] + b[1]*time
        # speed
        qd_t = b[1]*np.ones(len(time))
        # return a constant accel (zero)
        qdd_t = 0.0*np.ones(len(time))
        return q_t, qd_t, qdd_t

    def quintic_trajectory(self, q0, qf, qd0, qdf, qdd0, qddf, total_time, precision):
        #  consider the case in which the final time is zero, in this case, return the final targets
        if total_time < self.delta_time:
            q = np.array([qf]).T
            qd = np.array([qdf]).T
            qdd = np.array([qddf]).T
            total_time = np.array([total_time])
            return q, qd, qdd, total_time
        q = []
        qd = []
        qdd = []
        for i in range(self.DOF):
            qi, qdi, qddi, t = self.quintic_trajectory_single_joint((q0[i], qf[i]),
                                                                    (qd0[i], qdf[i]),
                                                                    (qdd0[i], qddf[i]),
                                                                    (0, total_time),
                                                                    precision)
            q.append(qi)
            qd.append(qdi)
            qdd.append(qddi)
        q = np.array(q)
        qd = np.array(qd)
        qdd = np.array(qdd)
        return q, qd, qdd, t

    def quintic_trajectory_single_joint(self, q, qd, qdd, t, precision):
        """
        function[q_t, qd_t, qdd_t, time, k] = fifth_order(q, qd, qdd, t, delta_t)
        """
        delta_t = self.delta_time
        A = np.array([[1, t[0], t[0] ** 2, t[0] ** 3, t[0] ** 4, t[0] ** 5],
                      [1, t[1], t[1] ** 2, t[1] ** 3, t[1] ** 4, t[1] ** 5],
                      [0, 1, 2 * t[0], 3 * t[0] ** 2, 4 * t[0] ** 3, 5 * t[0] ** 4],
                      [0, 1, 2 * t[1], 3 * t[1] ** 2, 4 * t[1] ** 3, 5 * t[1] ** 4],
                      [0, 0, 2, 6 * t[0], 12 * t[0] ** 2, 20 * t[0] ** 3],
                      [0, 0, 2, 6 * t[1], 12 * t[1] ** 2, 20 * t[1] ** 3]])
        # A * k = b
        b = np.array([q[0], q[1], qd[0], qd[1], qdd[0], qdd[1]])

        # k = [k[1] k[2] k[3] k[4] k[5] k[6]]
        k = np.dot(np.linalg.inv(A), b.T)
        n = int(np.floor((t[1] - t[0]) / delta_t))
        t = np.linspace(t[0], t[1], n + 1)
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
        if precision:
            n_settle = int(self.settle_time/self.delta_time)
        else:
            n_settle = 0
        # append the last values!!
        for i in range(n_settle):
            q.append(q[-1])
            qd.append(qd[-1])
            qdd.append(qdd[-1])
            t = np.append(t, ti + (i + 1) * delta_t)
        q = np.array(q)
        qd = np.array(qd)
        qdd = np.array(qdd)
        return q, qd, qdd, t

    def plot_planned_trajectories(self, qt, qdt, qddt, t, n_joints):
        plt.figure()
        for i in range(0, n_joints):
            plt.plot(t, qt[i, :], label='q' + str(i + 1))
        plt.legend()
        plt.title('JOINT TRAJECTORIES (rad, m)')
        plt.show(block=True)
        # Now plot speeds
        for i in range(0, n_joints):
            plt.plot(t, qdt[i, :], label='qd' + str(i + 1))
        plt.legend()
        plt.title('JOINT VELOCITIES (rad/s, m/s)')
        plt.show(block=True)
        # Now plot speeds
        for i in range(0, n_joints):
            plt.plot(t, qddt[i, :], label='qdd' + str(i + 1))
        plt.legend()
        plt.title('JOINT ACCELERATIONS (rad/s/s, m/s/s)')
        plt.show(block=True)

    def plot_planned_path(self, q_path):
        # joint position and references
        plt.figure()
        for i in range(self.DOF):
            plt.plot(range(q_path.shape[1]), q_path[i, :], label='q' + str(i + 1), linewidth=4)
        plt.title('PLANNED TRAJECTORIES')
        plt.legend()
        plt.show()

