#!/usr/bin/env python
# encoding: utf-8
"""
Base Robot Class

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import path_planning_line_factors, filter_path, time_trapezoidal_path_i, path_trapezoidal_i, \
    path_planning_line_constant_speed
# from artelib.plottools import plot_vars, plot, plot3d
# from artelib.tools import compute_w_between_orientations, euler2rot, rot2quaternion, buildT, compute_w_between_R, \
#     null_space, diff_w_central, w_central, null_space_projector, compute_kinematic_errors, rot2euler, quaternion2rot, \
#     q2euler, buildT
import matplotlib.pyplot as plt
# from robots.objects import ReferenceFrame
from artelib.tools import angular_w_between_quaternions
from artelib.vector import Vector


class Robot():
    def __init__(self, simulation):
        self.simulation = simulation
        self.serialrobot = None
        # a list of joint handles to move the robot
        self.joints = None
        self.max_joint_speeds = None
        self.joint_ranges = None
        # parameters of the inverse kinematics algorithm
        self.max_iterations_inverse_kinematics = None
        # max iterations to achieve a joint target in coppelia
        self.max_iterations_joint_target = None
        # admit this error in |q|2
        self.epsilonq = None
        # current robot joint positions. Initt to zeros
        self.q_current = None
        self.q_path = []
        self.qd_path = []

        # max errors during computation of inverse kinematics
        self.max_error_dist_inversekinematics = None
        self.max_error_orient_inversekinematics = None
        self.do_apply_joint_limits = None
        # wait these iterations before a WARNING is issued
        self.max_iterations_joint_target = 100

        # base reference system transformation
        self.T0 = HomogeneousMatrix(np.eye(4))
        # end tool tcp transformation
        self.Ttcp = HomogeneousMatrix(np.eye(4))

    def set_T0(self, T0):
        """
        Set the reference frame of the base of the robot
        """
        self.T0 = T0

    def set_TCP(self, Ttcp):
        """
        Set the TCP transformation
        """
        self.Ttcp = Ttcp

    def set_joint_target_velocities(self, qd):
        """
        CAUTION: this function does only work if the control mode is set to velocity.
        Set the arm joint speeds
        :param qd: joint speeds rad/s
        :return:
        """
        for i in range(len(qd)):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], float(qd[i]))

    def control_joint_positions(self, q):
        """
        A very simple joint control
        CAUTION: this function does only work if the joint control mode is set to Position-control.
        Set the arm joint positions
        :param q: joint position rad
        :return:
        """
        for i in range(len(q)):
            self.simulation.sim.setJointTargetPosition(self.joints[i], q[i])

        for i in range(500):
            self.wait()
            qc = self.get_joint_positions()
            e = np.linalg.norm(q-qc)
            if e < self.epsilonq:
                break


    def get_joint_positions(self):
        q_actual = np.zeros(len(self.joints))
        n = len(self.joints)
        for i in range(0, n):
            q_actual[i] = self.simulation.sim.getJointPosition(self.joints[i])
        return q_actual

    def get_joint_speeds(self):
        qd_actual = np.zeros(len(self.joints))
        n = len(self.joints)
        for i in range(0, n):
            qd_actual[i] = self.simulation.sim.getJointVelocity(self.joints[i])
        return qd_actual

    def moveAbsJ(self, q_target, qdfactor=1.0, precision=True, endpoint=True):
        """
        Commands the robot to the specified joint target positions.
        The targets are filtered and the robot is not commanded whenever a single joint is out of range.
        A path is planned considering the qdmax factor which ranges from 0 (zero speed) to 1.0 (full joint speed).
        """
        # remove joints out of range and get the closest joint
        total, partial = self.check_joints(q_target)
        if total:
            q_current = self.get_joint_positions()
            delta = np.linalg.norm(q_target-q_current)
            # only plan if some of the joints are very far from the desired q_target
            if delta > self.epsilonq:
                qs, qds = self.path_plan_isochronous_trapezoidal(q_target, qdfactor=qdfactor, endpoint=endpoint)
                # apply the computed profile in joint and speeds
                self.apply_speed_joint_control(qs, qds)
                if precision:
                    self.apply_position_joint_control(qs[:, -1], precision=True)
                    self.command_zero_target_velocities()
            else:
                self.apply_position_joint_control(q_target, precision=True)
                self.command_zero_target_velocities()
            q_current = self.get_joint_positions()
            delta = np.linalg.norm(q_target - q_current)
            print('Error final q: ', q_target - q_current)
            print('Error final q: ', delta)
        else:
            print('moveABSJ ERROR: target joints out of range')

    def moveJ(self, target_position, target_orientation, qdfactor = 1.0, endpoint=True, extended=True, precision=True):
        """
        Commands the robot to a target position and orientation.
        All solutions to the inverse kinematic problem are computed. The closest solution to the
        current position of the robot q0 is used
        Parameters:
            qdmax [0, 1.0]: a ratio of the max speed for all joints.
            target_point: if a target point, stop all joints when finished the trajectory.
            extended: Ask the inverse kinematic algorithm to include solutions out of the [-pi, pi] range
        """
        q_current = self.get_joint_positions()
        # resultado filtrado. Debe ser una matriz 6xn_movements
        # CAUTION. This calls the inverse kinematic method of the derived class

        q_target = self.inversekinematics(target_position=target_position,
                                          target_orientation=target_orientation, extended=extended, q0=q_current)
        if len(q_target) == 0:
            print('ERROR COMPUTING INVERSE KINEMATICS')
            print('Please check that the specified target point is reachable')
            return q_target
        # remove joints out of range and get the closest joint
        # filter a valid path from q_current to any of the solutions in q_target
        qs = filter_path(self, q_current, [q_target])
        # check if there is at least a single valid solution
        # print(qs.size)
        if qs.size == 0:
            print('CAUTION: NO VALID SOLUTIONS FOUND! IS THE POSITIONS/ORIENTATION REACHABLE?')
            raise Exception('INVERSE KINEMATICS ERROR. IS THE TARGET REACHABLE?')
        q_target = qs[:, 0]
        qs, qds = self.path_plan_isochronous_trapezoidal(q_target, qdfactor=qdfactor, endpoint=endpoint)
        # apply the computed profile in joint and speeds
        self.apply_speed_joint_control(qs, qds)
        if precision:
            self.apply_position_joint_control(qs[:, -1], precision=True)
            self.command_zero_target_velocities()
        # if endpoint:
        #     self.command_zero_target_velocities()

    def moveL(self, target_position, target_orientation, endpoint=False, extended=True, vmax=0.8, wmax=0.2, precision=True):
        q0 = self.get_joint_positions()
        # resultado filtrado. Debe ser una matriz 6xn_movements
        qs, qds = self.inversekinematics_line(q0=q0, target_position=target_position,
                                              target_orientation=target_orientation,
                                              extended=extended, vmax=vmax, wmax=wmax)
        self.apply_speed_joint_control(qs, qds)
        if precision:
            self.apply_position_joint_control(qs[:, -1], precision=True)
            self.command_zero_target_velocities()

    def moveAbsPath(self, q_path, qdfactor=1.0, precision=True, endpoint=True):
        """
        Commands the robot to a specified set of paths.
        """
        n_movements = q_path.shape[1]
        for i in range(n_movements):
            self.moveAbsJ(q_target=q_path[:, i], qdfactor=qdfactor, precision=precision, endpoint=endpoint)

    def command_zero_target_velocities(self):
        for i in range(len(self.joints)):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], 0)

    def apply_speed_joint_control(self, qs, qds):
        """
        Apply a set of computed speeds profiles to the joints
        try to follow qs by applying a corrected version of qds
        caution: additive control considering the error on each of the joints
        qs and qds are the target joint and speed references to be followed
        """
        delta_time = 0.05
        n_samples = qs.shape[1]
        # closed loop part
        # using a global pid control for all joints
        kpe = 5.5
        kde = 0.4
        kps = 0.2
        qreal = []
        qdreal = []
        eqi_1 = 0
        for i in range(n_samples):
            q_current = self.get_joint_positions()
            qd_current = self.get_joint_speeds()
            self.q_path.append(q_current)
            self.qd_path.append(qd_current)
            qreal.append(q_current)
            qdreal.append(qd_current)
            # correct by a small amount based on the error
            qi = qs[:, i]
            qdi = qds[:, i]
            eqi = qi-q_current
            # eqdi = qdi-qd_current
            deqi = (eqi-eqi_1)/delta_time
            eqi_1 = eqi
            # add a small quantity based on the error on each joint
            # (feedforward control with compensation)
            u = qdi + kpe*eqi + kde*deqi #+ kps*eqdi + kpe*eqi
            self.set_joint_target_velocities(u)
            self.simulation.client.step()

        # last speed command
        qdi = qds[:, i]
        u = qdi #+ kps*eqdi + kpe*eqi
        self.set_joint_target_velocities(u)
        self.simulation.client.step()

        q_current = self.get_joint_positions()
        qd_current = self.get_joint_speeds()
        self.q_path.append(q_current)
        self.qd_path.append(qd_current)
        qreal.append(q_current)
        qdreal.append(qd_current)

        # plot trajectories!
        # qreal = np.array(qreal).T
        # qdreal = np.array(qdreal).T
        # qreal = qreal[0, :]
        # qdreal = qdreal[0, :]
        # qs = qs[0, :]
        # qds = qds[0, :]

        # plt.plot(range(n_samples), qs)
        # plt.plot(range(n_samples+1), qreal)
        # plt.show()
        # #
        # plt.plot(range(n_samples), qds)
        # plt.plot(range(n_samples + 1), qdreal)
        # plt.show()

    def apply_position_joint_control(self, q_target, precision=True):
        """
        Apply a set of computed speeds profiles to the joints
        try to follow qs by applying a corrected version of qds
        caution: additive control considering the error on each of the joints
        """
        if precision:
            delta_threshold = 0.001
        else:
            delta_threshold = 0.1
        k = 5.5
        for i in range(50):
            q_current = self.get_joint_positions()
            qd_current = self.get_joint_speeds()
            self.q_path.append(q_current)
            self.qd_path.append(qd_current)
            e = q_target - q_current
            delta = np.linalg.norm(e)
            if delta < delta_threshold:
                break
            u = k * e
            self.set_joint_target_velocities(u)
            self.simulation.client.step()

    def get_min_distance_to_objects(self):
        """
        Caution: a signal must have been added to the Coppelia Simulation (called distance_to_sphere)
        """
        error, distance = self.simulation.sim.getFloatSignal('min_distance_to_objects')
        return distance

    def wait(self, steps=1):
        self.simulation.wait(steps=steps)

    def wait_time(self, seconds):
        self.simulation.wait_time(seconds=seconds)

    def wait_till_joint_position_is_met(self, q_target):
        for i in range(self.max_iterations_joint_target):
            q_actual = self.get_joint_positions()
            error = np.linalg.norm(q_target-q_actual)
            # print('Current error is:', error)
            # print('n_iterations: ', i)
            if error < self.epsilonq:
                return
            self.simulation.wait()
            # sim.simxSynchronousTrigger(clientID=self.clientID)

        print('ERROR, joint position could not be achieved, try increasing max_iterations')
        print('Errors (q)')
        print(q_target-q_actual)
        print('Total error is:', np.linalg.norm(q_target-q_actual))

    def get_symbolic_jacobian(self, q):
        # calling derived class get_jacobian
        # should be implemented at the UR5, UR10 classes etc.
        return self.get_symbolic_jacobian(q)

    def compute_manipulability(self, q):
        [J, _, _] = self.manipulator_jacobian(q)
        manip = np.sqrt(np.linalg.det(np.dot(J, J.T)))
        return manip

    def directkinematics(self, q):
        A = self.serialrobot.directkinematics(q)
        T = self.T0*A*self.Ttcp
        return T

    def dh(self, q, i):
        A = self.serialrobot.dh(q, i)
        return A

    def compute_time(self, Tcurrent, Ttarget, vmax=1.0):
        """
        Compute the movement that allows to bring Tcurrent to Ttarget with a given linear max speed
        """
        # current position of the end effector and target position
        p_current = Tcurrent[0:3, 3]
        p_target = Ttarget[0:3, 3]
        vref = np.array(p_target-p_current)
        dist = np.linalg.norm(vref)
        # total time to complete the movement given vmax
        total_time = dist/vmax
        return total_time

    def check_joints(self, q):
        """
        Check that each joint is within range.
        Returns True if all joints are within range
        Returns False if not.
        Finally, an array with the valid indexes are returned
        """
        valid = True
        valid_indexes = []
        for i in range(0, len(q)):
            # greater than min and lower than max
            if (self.joint_ranges[0, i] <= q[i]) and (self.joint_ranges[1, i] >= q[i]):
                valid_indexes.append(True)
                continue
            else:
                # print(30*'*')
                # print('JOINT ERROR: RANGE ERROR! Joint: q', i+1, ' is out of range')
                # print(30 * '*')
                valid = False
                valid_indexes.append(False)
        return valid, valid_indexes

    def filter_joint_limits(self, q):
        """
        Returns the solutions in q (by columns) that are within the joint ranges.
        Two behaviours are expected:
            a) In typical industrial robots (e. g. the IRB140) a 6x8 matrix stores all solutions,
            being each column a different solution.
            b) In other robots, such as the UR5 a
        """
        sh = q.shape
        q_in_range = []
        # filter unidimensional array
        if len(sh) == 1:
            total, partial = self.check_joints(q)
            if total:
                q_in_range.append(q)
            q_in_range = np.array(q_in_range).T
        # filter bidimensional array
        elif len(sh) == 2:
            n_valid_solutions = q.shape[1]
            for i in range(n_valid_solutions):
                qi = q[:, i]
                total, partial = self.check_joints(qi)
                if total:
                    q_in_range.append(qi)
            q_in_range = np.array(q_in_range).T
        return q_in_range

    def apply_joint_limits(self, q):
        """
        the value of qi will be saturated to the max or min values as specified in self.joint_ranges

        """
        # out_of_range states whether any of the joints is saturated (is going out of range)
        out_of_range = False
        for i in range(0, len(q)):
            # greater than min and lower than max --> then saturate
            if q[i] < self.joint_ranges[0, i]:
                q[i] = self.joint_ranges[0, i]
                out_of_range = True
            elif q[i] > self.joint_ranges[1, i]:
                q[i] = self.joint_ranges[1, i]
                out_of_range = True
        return q, out_of_range

    def check_speed(self, qd):
        """
        Checks that all joints speeds are within its limits.
        In addition, a corrected qd is returned that scales down the whole qd vector by a common constant.
        Please take into account that if qd is close to inf values, the returned vector will not meet any kinematic
        constrain.
        """
        # check that the array is finite
        check_nan = np.isnan(qd).any()
        check_inf = np.isinf(qd).any()
        if check_nan or check_inf:
            print(30 * '*')
            print('JOINT ERROR: SPEED IS INF OR NAN!')
            print('Setting speed to zero')
            print(30 * '*')
            return np.zeros(len(qd)), False, False
        # print('Joint speed norm: ', np.linalg.norm(qd))
        valid = True
        valid_indexes = []
        diffs = []
        ctes = []
        # corrected speed
        for i in range(0, len(qd)):
            diff = self.max_joint_speeds[i] - np.abs(qd[i])
            diffs.append(np.abs(diff))
            ctes.append(self.max_joint_speeds[i]/(0.01 + np.abs(qd[i])))
            # greater than min and lower than max
            if diff < 0:
                print(30*'*')
                print('JOINT ERROR: MAX SPEED!. Joint: q', i + 1, ' has speed above its maximum.')
                print(30*'*')
                valid = False
                valid_indexes.append(False)
            else:
                valid_indexes.append(True)
        # accomodate speed
        if not valid:
            cte = np.min(ctes)
            qd_corrected = np.dot(cte, qd)
        else:
            qd_corrected = qd
        return qd_corrected, valid, valid_indexes

    def inversekinematics_line_simple(self, q0, target_position, target_orientation, vmax=0.7, wmax=0.2, extended=True):
        """
        The end effector should follow a line in task space to reach target position and target orientation.
        A number of points is interpolated along the line, according to the speed vmax and simulation time
        (delta_time).
        The same number or points are also interpolated in orientation.
        Caution. target_orientationQ is specified as a quaternion
        """
        Ti = self.directkinematics(q0)
        target_positions, target_orientations = path_planning_line_constant_speed(Ti.pos(), Ti.R(), target_position, target_orientation,
                                                                   linear_speed=vmax, angular_speed=wmax)
        q_path = []
        # start joint position
        q = q0
        # now try to reach each target position on the line
        for i in range(len(target_positions)):
            q = self.inversekinematics(target_position=target_positions[i],
                                       target_orientation=target_orientations[i],
                                       q0=q, extended=extended)
            q_path.append(q)
        #  IMPORTANT:  q_path includes, for each time step, all possible solutions of the inverse kinematic problem.
        # for example, q_path will be a list with n movements. Each element in the list, is, again, a list including
        # all possible soutions for the inverse kinematic problem of that particular position and orientaition
        q_path = filter_path(self, q0, q_path)
        return q_path

    def inversekinematics_line(self, q0, target_position, target_orientation, vmax=0.7, wmax=0.2, extended=True):
        """
        Computing a trapezoidal speed profile over the line.

        path_planning_line is computed based on a set of displacements in [0, 1]

        """
        delta_time = 0.05
        Ti = self.directkinematics(q0)
        if isinstance(target_position, Vector):
            # print(Ti.pos())
            # print(target_position.pos())
            d = np.linalg.norm(Ti.pos()-target_position.pos())
        elif isinstance(target_position, list):
            d = np.linalg.norm(Ti.pos() - np.array(target_position))
        else:
            d = np.linalg.norm(Ti.pos() - target_position)
        # compute time using a trapezoidal profile (as in a single joint i)
        t_total_planning = time_trapezoidal_path_i(0.0, d, 0.0, vmax, endpoint=True)
        # compute the time, position and speed of the joint considering a trapezoidal profile (as in a single joint)
        t, dt, vt = path_trapezoidal_i(0, d, 0, t_total_planning, endpoint=True)
        # plt.plot(t, dt)
        # plt.show()
        # plt.plot(t, vt)
        # plt.show()
        # a 0 to 1 factor of  interpolation. Use the normalized displacementes
        factors = dt/d
        target_positions, target_orientations = path_planning_line_factors(Ti.pos(), Ti.R(),
                                                                     target_position, target_orientation, factors=factors)
        q_path = []
        # start joint position
        q = q0
        # now try to reach each target position on the line
        for i in range(len(factors)):
            q = self.inversekinematics(target_position=target_positions[i],
                                       target_orientation=target_orientations[i],
                                       q0=q, extended=extended)
            q_path.append(q)
        #  IMPORTANT:  q_path includes, for each time step, all possible solutions of the inverse kinematic problem.
        # for example, q_path will be a list with n movements. Each element in the list, is, again, a list including
        # all possible soutions for the inverse kinematic problem of that particular position and orientaition
        q_path = filter_path(self, q0, q_path)
        # compute differential speed
        qd_path = [np.zeros_like(q0)]
        for i in range(len(factors)-1):
            qi = q_path[:, i]
            qj = q_path[:, i+1]
            qd = (qj - qi)/delta_time
            qd_path.append(qd)
        q_path = np.array(q_path)
        qd_path = np.array(qd_path).T

        # # now, given, q_path, compute the joints speed based on the Jacobian
        # v= target_position.pos() - Ti.pos()
        # unit_v = v / np.linalg.norm(v)
        # Qcurrent = Ti.Q()
        # Qtarget = target_orientation.Q()
        # w = angular_w_between_quaternions(Qcurrent, Qtarget, t_total_planning)
        #
        # for i in range(len(q_path)):
        #     qi = q_path[i]
        #     vi = vt[i]*unit_v
        #     wi = w
        #     J = self.manipulator_jacobian(qi)
        # plt.plot(t, q_path.T)
        # plt.show()
        # plt.plot(t, qd_path.T)
        # plt.show()
        return q_path, qd_path

    def plot_trajectories(self):
        plt.figure()
        q_path = np.array(self.q_path)
        qd_path = np.array(self.qd_path)
        if len(q_path) == 0:
            return
        sh = q_path.shape
        for i in range(0, sh[1]):
            plt.plot(q_path[:, i], label='q' + str(i + 1))
        plt.legend()
        plt.title('JOINT TRAJECTORIES (rad, m)')
        plt.show(block=True)
        # Now plot speeds
        for i in range(0, sh[1]):
            plt.plot(qd_path[:, i], label='qd' + str(i + 1))
        plt.legend()
        plt.title('JOINT VELOCITIES (rad/s, m/s)')
        plt.show(block=True)

    def get_trajectories(self):
        return self.q_path

    def manipulator_jacobian(self, q):
        """
        Compute the manipulator Jacobian for the current joint position vector q.
        """
        # T = self.serialrobot.directkinematics(q)
        # number  of  DOF
        n = len(q)
        # base rotation matrix
        # R0 = np.eye(3)
        R0 = self.serialrobot.T0[0:3, 0:3]
        # Z vector on   each     reference     system
        z0 = np.array([0, 0, 1])

        # compute zi vectors. We first  start   by computing the   zi vectors   of    each    % reference    system
        # from z0, z1, z2..., z_{n - 1}. Note that the last  rotational (translational)  joint  acts   on z_{n - 1}
        # Array  to   store    every     z     vector
        z = []

        # this loop  computes vectors from z0 to z_{n - 1}
        for i in range(n):
            zi = np.dot(R0, z0)
            # store the  vector in a list
            z.append(zi)
            # compute  the  DH    transformation   matrix   from system  i - 1     to    system    i
            A = self.serialrobot.dh(q, i)
            # obtain  now    the    global transformation    by     postmultiplying     the rotational
            #     part. In     the     following    iteration   we     include    the    last    DH transformation
            R0 = np.dot(R0, A[0:3, 0:3])
        z = np.array(z)
        z = z.T

        # compute  p{i - 1}n *  vectors   that  represent  the   position   of   the    end effector in the  {i - 1}
        #  reference    system. Please  note  that  the  following code is not optimal (at all), the  total
        # transformation  matrix    should  be  computed in the    loop   above. However, we  compute  it   now   using
        # the directkinematic function for this particular  robot. Again, the    DH  matrices   are   computed    again.
        T = self.serialrobot.directkinematics(q)
        pn = np.zeros((3, n))
        Ti = self.serialrobot.T0
        v = Ti[0:3, 3]
        for i in range(n):
            pn[:, i] = T[0: 3, 3] - v
            A = self.serialrobot.dh(q, i)
            Ti = np.dot(Ti, A)
            # v is the origin  of  the   ith  reference  system in base   coordinates
            v = Ti[0:3, 3]
        # nowcompute conventional Jacobian
        J = np.zeros((6, n))
        for i in range(n):
            # rotational
            if self.serialrobot.get_link_type(i) == 'R':
                J[:, i] = np.concatenate((np.cross(z[:, i], pn[:, i]), z[:, i]))
            else: # translational   joint
                J[:, i] = np.concatenate((z[:, i], np.zeros((3, 1))))
        return J, J[0:3, :], J[3:6, :]

    def path_plan_isochronous(self, q_current, q_target, qdmax):
        """
        Plan an isochronous path in joint coordinates considering only a continuous speed.
        The slowest time is computed based on the total joint movement (rad) and the joint speeds.
        """
        # time at full speed
        t_times = np.array(q_target)-np.array(q_current)
        for i in range(len(self.joints)):
            t_times[i] = np.abs(t_times[i])/(qdmax*self.max_joint_speeds[i])
        # find the slowest
        t_planning = np.amax(t_times)
        # compute the max number of samples
        n_samples = int(np.round(t_planning/0.05))
        # plan speeds (constant speed), beware of the simplification
        qds = []
        # plan at a factor of the max
        qdi = (1/t_planning)*(np.array(q_target)-np.array(q_current))
        for i in range(n_samples):
            qds.append(qdi)
        # plan joints considering constant speed at each joint
        qs = []
        for i in range(len(self.joints)):
            qit = np.linspace(q_current[i], q_target[i], n_samples)
            qs.append(qit)
        qs = np.array(qs)
        qds = np.array(qds).T
        return qs, qds

    def path_plan_isochronous_trapezoidal(self, q_target, qdfactor, endpoint):
        """
        Plan an isochronous path in joint coordinates considering only a continuous speed.
        The slowest time is computed based on the total joint movement (rad) and the joint speeds.
        endpoint = True --> stopping (zero speed) at q_target
        endpoint = False --> will keep speed
        """
        # get current positions and speeds
        q_current = self.get_joint_positions()
        qd_current = self.get_joint_speeds()
        # find the time to complete the movement considering that
        # each joint works at a factor of its max speed
        t_times = []
        for i in range(len(self.joints)):
            ttotali = time_trapezoidal_path_i(q_current[i], q_target[i], qd_current[i], qdfactor*self.max_joint_speeds[i], endpoint=endpoint)
            t_times.append(ttotali)
        t_times = np.array(t_times)
        # find max speed an set as global time for planning
        t_total_planning = np.amax(t_times)
        qs = []
        qds = []
        for i in range(len(self.joints)):
            t, qti, qdti = path_trapezoidal_i(q_current[i], q_target[i], qd_current[i],
                                              t_total_planning, endpoint=endpoint)
            qs.append(qti)
            qds.append(qdti)
            # plt.plot(t, qti)
            # plt.show()
            # plt.plot(t, qdti)
            # plt.show()
        qs = np.array(qs)
        qds = np.array(qds)
        # plt.plot(t, qs.T)
        # plt.show()
        # plt.plot(t, qds.T)
        # plt.show()
        return qs, qds
