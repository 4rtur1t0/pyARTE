#!/usr/bin/env python
# encoding: utf-8
"""
Base Robot Class

@Authors: Arturo Gil
@Time: April 2021
"""
import matplotlib.pyplot as plt
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.jacobian import compute_jacobian_manipulator_function
from artelib.parameters import CoppeliaJoints
from artelib.path_planning import path_planning_line_factors, build_path_by_closest, \
    interpolate_targets_trapezoidal_line, get_closest_to, compute_qpath_along_line
from artelib.robot_state import RobotState
from artelib.trajectory_planner import TrajectoryPlanner
from artelib.control import Controller
from helpers.print_errors import ErrorPrint
from abc import ABC, abstractmethod


class Robot(ABC):
    def __init__(self, simulation, DOF, name, base_name, joint_name, joint_parameters, serial_parameters,
                 control_parameters, dynamic_parameters, frame):
        """
        CAUTION: EVERY ROBOT SERIAL IS COMMANDED IN TORQUE/FORCE MODE, THUS, the method
        self.set_control_to_force_mode() is called.
        Each robot joint may be configured by default in speed control in Coppelia. In this way, you can
        start/stop the simulation while the robot is stopped.
        """
        self.simulation = simulation
        self.DOF = DOF
        self.name = name
        self.joint_parameters = joint_parameters  #(joint ranges, joint speeds)
        self.serial_parameters = serial_parameters
        self.dynamic_parameters = dynamic_parameters
        self.control_parameters = control_parameters
        # store the current state, saved robot state, torques, time, etc.
        self.robot_state = RobotState(self)
        # a 3D reference frame to show target points
        self.frame = frame
        # base reference system transformation
        self.T0 = HomogeneousMatrix(np.eye(4))
        # end tool tcp transformation
        self.Ttcp = HomogeneousMatrix(np.eye(4))
        # the trajectory planner object
        self.planner = TrajectoryPlanner(joint_parameters=joint_parameters,
                                         delta_time=self.simulation.delta_time,
                                         settle_time=joint_parameters.settle_time)
        # max difference in planning a line in space
        self.q_line_planning_threshold = np.pi/4
        # the control object
        self.controller = Controller(self, control_parameters=control_parameters)

        # a class to print errors and warnings with colors
        self.error_print = ErrorPrint()
        self.coppelia_joints = CoppeliaJoints(base_name=base_name, joint_name=joint_name)
        self.last_qref = np.zeros(self.DOF)
        self.wrist_sing = []

    @abstractmethod
    def inverse_kinematics(self, target_position, target_orientation, extended=False, q0=None):
        """Derived classes must implement this method"""
        pass

    @abstractmethod
    def inverse_dynamics(self, q, qd, qdd):
        """Derived classes must implement this method"""
        pass

    @abstractmethod
    def check_can_be_reached(self, target_positions, target_orientations):
        """
        Derived classes must implement this method
        Check that every target point can be reached
        """
        pass

    def start(self):
        armjoints = []
        # Get the handles of the relevant objects
        base_name = self.coppelia_joints.base_name
        joint_name = self.coppelia_joints.joint_name
        robotbase = self.simulation.sim.getObject(base_name)
        for i in range(self.DOF):
            qi = self.simulation.sim.getObject(base_name + '/' + joint_name + str(i + 1))
            armjoints.append(qi)
        # initialize the elements that must access the robot joints
        self.coppelia_joints.joint_handlers = armjoints
        self.robot_state.joint_handlers = armjoints
        # SET CONTROL MODE TO FORCE/TORQUE
        self.set_control_to_force_mode()
        q = np.zeros(self.DOF)
        self.moveAbsJ(q_target=q, precision=False)

    def stop_joints(self):
        """
        Set every joint to speed control mode and set the speed to zero
        """
        self.set_control_to_velocity_mode()
        # Now stop
        self.set_joint_target_speeds(0.0*np.ones(self.DOF))
        self.wait(1)

    def set_control_to_force_mode(self):
        # set the joints to control force
        for i in range(self.DOF):
            self.simulation.sim.setObjectInt32Param(self.coppelia_joints.joint_handlers[i],
                                                    self.simulation.sim.jointintparam_dynctrlmode,
                                                    self.simulation.sim.jointdynctrl_force)

    def set_control_to_velocity_mode(self):
        for i in range(self.DOF):
            self.simulation.sim.setObjectInt32Param(self.coppelia_joints.joint_handlers[i],
                                                    self.simulation.sim.jointintparam_dynctrlmode,
                                                    self.simulation.sim.jointdynctrl_velocity)

    def get_joint_positions(self):
        q_actual = np.zeros(self.DOF)
        n = len(self.coppelia_joints.joint_handlers)
        for i in range(0, n):
            q_actual[i] = self.simulation.sim.getJointPosition(self.coppelia_joints.joint_handlers[i])
        return q_actual

    def get_joint_speeds(self):
        qd_actual = np.zeros(len(self.coppelia_joints.joint_handlers))
        n = len(self.coppelia_joints.joint_handlers)
        for i in range(0, n):
            qd_actual[i] = self.simulation.sim.getJointVelocity(self.coppelia_joints.joint_handlers[i])
        return qd_actual

    def set_joint_target_speeds(self, target_speeds):
        for i in range(self.DOF):
            self.simulation.sim.setJointTargetVelocity(self.coppelia_joints.joint_handlers[i],
                                                       target_speeds[i])

    def apply_torques(self, tau):
        # self.apply_torques(tau)
        for i in range(len(tau)):
            self.simulation.sim.setJointTargetForce(self.coppelia_joints.joint_handlers[i], tau[i])

    def get_state_time(self):
        return self.robot_state.get_state_time()

    def get_state(self):
        return self.robot_state.get_state()

    def save_state(self):
        self.robot_state.save_state()

    def reset_state(self):
        self.robot_state.reset_state()

    def plot_states(self):
        self.robot_state.plot_states()

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

    def moveAbsJ(self, q_target, precision=True, plot=False, true_trapezoidal=False, speed_factor=1.0):
        """
        Commands the robot to the specified joint target positions.
        The targets are filtered and the robot is not commanded whenever a single joint is out of range.
        A path is planned considering the qdmax factor which ranges from 0 (zero speed) to 1.0 (full joint speed).
        The true_trapezoidal option indicates the planner to create a true trapezoidal speed profile. A smoother, yet
        not trapezoidal profile can be created with a partial approximation.
        Speed_factor can be used to give the robot higher or lower speeds.
        Usually, speed_factor should be 1.0. Use a speed_factor of 0.5, 0.1 if precise operation is needed.
        Also, use speed_factor = 3.0 if the simulation is slow.
        """
        # remove joints out of range and get the closest joint
        total, partial = self.check_joints(q_target)
        if not total:
            self.error_print.print('[ERROR]. moveAbsJ: one or more of the specified joints are out of range', 'red')
            self.error_print.print('The joint out of range says False:', 'red')
            print(partial)
            return
        # get the current joint values
        q_current = self.get_joint_positions()
        v_max = speed_factor*self.joint_parameters.max_joint_speeds
        a_max = speed_factor*self.joint_parameters.max_joint_accelerations
        # plan a true trapezoidal path
        if true_trapezoidal:
            qt, qdt, qddt, time = self.planner.trapezoidal_coordinated(s0=q_current,
                                                                       sf=q_target,
                                                                       v_max=v_max,
                                                                       a_max=a_max,
                                                                       precision=precision,
                                                                       plot=plot)
        else:
            qt, qdt, qddt, time = self.planner.trapezoidal_coordinated_approx(s0=q_current,
                                                                              sf=q_target,
                                                                              v_max=v_max,
                                                                              a_max=a_max,
                                                                              precision=precision,
                                                                              plot=plot)
        # perform control!
        self.controller.control(qt, qdt, qddt, plot=plot)
        # save last pose
        self.last_qref = q_target

    def moveAbsPath(self, q_path, plot=False, speed_factor=1.0, sampling=1, precision=False):
        """
        Commands the robot to a specified set of paths.
        """
        n_movements = q_path.shape[1]
        for i in range(0, n_movements, sampling):
            self.moveAbsJ(q_target=q_path[:, i], speed_factor=speed_factor, plot=plot, precision=precision)
        self.last_qref = q_path[:, -1]

    def moveJ(self, target_position, target_orientation, speed_factor=1.0, precision=True, extended=True):
        """
        Commands the robot to a target position and orientation.
        All solutions to the inverse kinematic problem are computed. The closest solution to the
        current position of the robot q0 is used
        Parameters:
            qdmax [0, 1.0]: a ratio of the max speed for all joints.
            target_point: if a target point, stop all joints when finished the trajectory.
            extended: Ask the inverse kinematic algorithm to include solutions out of the [-pi, pi] range
        """
        if self.frame:
            self.frame.show_target_point(target_position=target_position, target_orientation=target_orientation)
        q_current = self.get_joint_positions()
        # CAUTION. This calls the inverse kinematic method of the derived class
        q_target = self.inverse_kinematics(target_position=target_position,
                                           target_orientation=target_orientation, extended=extended)
        if len(q_target) == 0:
            self.error_print.print('ERROR COMPUTING INVERSE KINEMATICS', color='red')
            self.error_print.print('Please check that the specified target point is reachable', color='red')
            return q_target
        # filter solutions out of range
        q_target = self.filter_joint_limits(q_target)
        if len(q_target) == 0:
            self.error_print.print('ERROR COMPUTING INVERSE KINEMATICS. NO SOLUTIONS IN JOINT RANGE', color='red')
            self.error_print.print('Please check that the specified target point is reachable', color='red')
            return q_target
        # get the closest solution to the current joint positions
        # q_target = build_path_by_closest(q_current, [q_target])
        q_target = get_closest_to(q_current, q_target)
        # q must be a row!
        #q_target = q_target[:, 0]
        if q_target.size == 0:
            self.error_print.print('CAUTION: NO VALID SOLUTIONS FOUND! IS THE POSITIONS/ORIENTATION REACHABLE?',
                                   color='red')
            raise Exception('INVERSE KINEMATICS ERROR. IS THE TARGET REACHABLE?')
        # finally, move to the desired target
        self.moveAbsJ(q_target=q_target, speed_factor=speed_factor, precision=precision)
        self.last_qref = q_target

    def moveL(self, target_position, target_orientation, precision=False, extended=True, speed_factor=1.0, plot=False,
              debug=False):
        if self.frame:
            self.frame.show_target_point(target_position=target_position, target_orientation=target_orientation)
        # Interpolate positions and orientations along the line: a coordinated trapezoidal is computed considering
        # distance and angle of rotation
        target_positions, target_orientations, time = interpolate_targets_trapezoidal_line(self, target_position,
                                                                                           target_orientation,
                                                                                           speed_factor,
                                                                                           plot, precision)
        if debug:
            self.frame.show_target_points(target_positions=target_positions, target_orientations=target_orientations)
        # First check that all the points are reachable
        total, partial = self.check_can_be_reached(target_positions=target_positions,
                                                   target_orientations=target_orientations)
        if not total:
            self.error_print.print('moveL KINEMATIC ERROR: at least one point of the line is out of range', 'red')
            raise Exception
        # computing a path along a line using the closest inverse kinematic solution at each time step
        # the path is built by adding the closest solution of the inverse kinematic problem to move
        # form a target point to the next. ALSO: the solutions that are out of joint ranges are removed
        q_path = compute_qpath_along_line(self, target_positions, target_orientations, extended)
        # debug: observe whether the total path is followed by the robot.
        if debug:
            self.planner.plot_planned_path(q_path=q_path)
            self.moveAbsPath(q_path=q_path, speed_factor=10.0, sampling=1, precision=False)
        if (len(target_positions) != q_path.shape[1]) or q_path.shape[1] == 0:
            self.error_print.print('moveL KINEMATIC ERROR: no valid solutions could be found for some points on the workspace trajectory', 'red')
            raise Exception
        # check if any difference between consecutive joints values is found greater
        # than a threshold and issue an exception in that case
        total = self.check_path_absolute_differences(q_path=q_path)
        if not total:
            self.error_print.print('moveL KINEMATIC ERROR: one of the joints moved two much between consecutive poses.',
                                   'red')
            raise Exception
        # compute joint speed and acceleration approximately
        qd_path = np.gradient(q_path, self.simulation.delta_time, axis=1)
        qdd_path = np.gradient(qd_path, self.simulation.delta_time, axis=1)
        # perform control!
        self.controller.control(q_path, qd_path, qdd_path, plot=plot)
        self.last_qref = q_path[:, -1]

    def moveQuintic(self, q_target, total_time, precision=False, plot=False):
        self.last_qref = q_target
        # remove joints out of range and get the closest joint
        total, partial = self.check_joints(q_target)
        if not total:
            self.error_print.print('[ERROR]. moveQuintic: one or more of the specified joints are out of range', 'red')
            print('The joint out of range says False:', partial)
            return
        # get the current joint values
        q_current = self.get_joint_positions()
        # plan a trapezoidal path
        qt, qdt, qddt, time = self.planner.quintic_trajectory(q0=q_current,
                                                              qf=q_target,
                                                              qd0=np.zeros(self.DOF),
                                                              qdf=np.zeros(self.DOF),
                                                              qdd0=np.zeros(self.DOF),
                                                              qddf=np.zeros(self.DOF),
                                                              total_time=total_time,
                                                              precision=precision)
        # perform control!
        self.controller.control(qt, qdt, qddt, plot=plot)
        self.last_qref = q_target

    def wait(self, steps=1):
        """
        CAUTION: DO NOT CALL simulation.wait(), since the robot must be controlled at
        every time step
        """
        ref = self.last_qref
        for i in range(steps):
            self.moveQuintic(q_target=ref, total_time=self.simulation.delta_time)

    def wait_time(self, seconds):
        ref = self.last_qref
        t1 = self.simulation.sim.getSimulationTime()
        while True:
            t2 = self.simulation.sim.getSimulationTime()
            if (t2 - t1) >= seconds:
                break
            self.wait()
        self.last_qref = ref
        # self.simulation.wait_time(seconds=seconds)

    def get_symbolic_jacobian(self, q):
        # calling derived class get_jacobian
        # should be implemented at the UR5, UR10 classes etc.
        return self.get_symbolic_jacobian(q)

    def manipulator_jacobian(self, q):
        """
        Compute the manipulator Jacobian for the current joint position vector q.
        """
        J, Jv, Jw = compute_jacobian_manipulator_function(self, q)
        return J, Jv, Jw

    def compute_manipulability(self, q):
        [J, _, _] = self.manipulator_jacobian(q)
        manip = np.sqrt(np.linalg.det(np.dot(J, J.T)))
        return manip

    def directkinematics(self, q):
        A = self.serial_parameters.directkinematics(q)
        T = self.T0*A*self.Ttcp
        return T

    def dh(self, q, i):
        A = self.serial_parameters.dh(q, i)
        return A

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
            if (self.joint_parameters.joint_ranges[0, i] <= q[i]) and (self.joint_parameters.joint_ranges[1, i] >= q[i]):
                valid_indexes.append(True)
                continue
            else:
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
        # if isinstance(q, list)
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
            if q[i] < self.joint_parameters.joint_ranges[0, i]:
                q[i] = self.joint_parameters.joint_ranges[0, i]
                out_of_range = True
            elif q[i] > self.joint_parameters.joint_ranges[1, i]:
                q[i] = self.joint_parameters.joint_ranges[1, i]
                out_of_range = True
        return q, out_of_range

    def check_path_absolute_differences(self, q_path):
        # check if any large difference between consecutive joints values is found in the
        # path, if this is the case,
        # FUNCTION TO COMPUTE DIFFERENCES
        q_differences = np.diff(q_path, axis=1)
        q_differences = np.abs(q_differences)
        indices = np.where(q_differences > self.q_line_planning_threshold)[0]
        if len(indices) > 0:
            return False
        return True
        # if len(indices) > 0:
        #     self.error_print.print(
        #         'moveL KINEMATIC ERROR: one of the joints moved two much between consecutive poses. '
        #         'You may be entering a kinematic singularity',
        #         'red')
        #     self.error_print.print(
        #         'moveL KINEMATIC ERROR: please try decreasing the movement or changing the target point',
        #         'red')
        #     raise Exception

    # def check_speed(self, qd):
    #     """
    #     Checks that all joints speeds are within its limits.
    #     In addition, a corrected qd is returned that scales down the whole qd vector by a common constant.
    #     Please take into account that if qd is close to inf values, the returned vector will not meet any kinematic
    #     constrain.
    #     """
    #     # check that the array is finite
    #     check_nan = np.isnan(qd).any()
    #     check_inf = np.isinf(qd).any()
    #     if check_nan or check_inf:
    #         print(30 * '*')
    #         print('JOINT ERROR: SPEED IS INF OR NAN!')
    #         print('Setting speed to zero')
    #         print(30 * '*')
    #         return np.zeros(len(qd)), False, False
    #     # print('Joint speed norm: ', np.linalg.norm(qd))
    #     valid = True
    #     valid_indexes = []
    #     diffs = []
    #     ctes = []
    #     # corrected speed
    #     for i in range(0, len(qd)):
    #         diff = self.max_joint_speeds[i] - np.abs(qd[i])
    #         diffs.append(np.abs(diff))
    #         ctes.append(self.max_joint_speeds[i]/(0.01 + np.abs(qd[i])))
    #         # greater than min and lower than max
    #         if diff < 0:
    #             print(30*'*')
    #             print('JOINT ERROR: MAX SPEED!. Joint: q', i + 1, ' has speed above its maximum.')
    #             print(30*'*')
    #             valid = False
    #             valid_indexes.append(False)
    #         else:
    #             valid_indexes.append(True)
    #     # accomodate speed
    #     if not valid:
    #         cte = np.min(ctes)
    #         qd_corrected = np.dot(cte, qd)
    #     else:
    #         qd_corrected = qd
    #     return qd_corrected, valid, valid_indexes


def plot_joints(qis, q0, qi):
    plt.figure()
    n_solutions = qis.shape[1]
    plt.plot(range(6), q0, label='PREVIOUS')
    plt.plot(range(6), qi, label='CLOSEST')
    plt.legend()
    plt.show()

    for i in range(n_solutions):
        plt.plot(range(6), qis[:, i], label='solution ' + str(i))
    plt.legend()
    plt.show()
