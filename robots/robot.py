#!/usr/bin/env python
# encoding: utf-8
"""
Base Robot Class

@Authors: Arturo Gil
@Time: April 2021

"""
import sim
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.inverse_kinematics import delta_q
from artelib.path_planning import generate_target_positions, generate_target_orientations, \
    generate_target_orientations_Q, move_target_positions_obstacles, n_movements
from artelib.plottools import plot_vars, plot, plot3d
from artelib.tools import compute_w_between_orientations, euler2rot, rot2quaternion, buildT, compute_w_between_R, \
    null_space, diff_w_central, w_central, null_space_projector, compute_kinematic_errors, rot2euler, quaternion2rot, \
    q2euler, buildT
import matplotlib.pyplot as plt
import time
from PIL import Image, ImageOps

# standard Coppelia simulation time. PLEASE CHANGE THIS ACCORDINGLY
DELTA_TIME = 50.0/1000.0


class Robot():
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target, camera,
                 max_joint_speeds, joint_ranges, epsilonq=0.001):
        self.clientID = clientID
        self.wheeljoints = wheeljoints
        self.armjoints = armjoints
        self.base = base
        self.gripper = gripper
        self.end_effector = end_effector
        self.camera = camera
        self.target = target
        self.max_joint_speeds = max_joint_speeds
        self.joint_ranges = joint_ranges
        # parameters of the inverse kinematics algorith
        self.max_iterations_inverse_kinematics = 15000
        # max iterations to achieve a joint target in coppelia
        self.max_iterations_joint_target = 50
        # admit this error in |q|2
        self.epsilonq = epsilonq
        self.q_path = []

        # max errors during computation of inverse kinematics
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        self.do_apply_joint_limits = False

    def set_joint_target_velocities(self, qd):
        """
        CAUTION: this function does only work if the position control loop is disabled at every youbot armjoint.
        Set the arm joint speeds
        :param qd: joint speeds rad/s
        :return:
        """
        for i in range(len(qd)):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                       targetVelocity=qd[i],
                                                       operationMode=sim.simx_opmode_oneshot)

    def set_joint_target_positions(self, q_target, precision=True):
        """
        CAUTION: this function may only work if the "position control loop" is enabled at every arm joint.
        :param precision: whether to wait for Coppelia until que joint values are attained with precision
                    precision=True: --> the method self.wait_till_joint_position_is_met is called. This method
                    checks, at each simulation time, whether the specified joint values q_target have been achieved.
        :return: None
        """
        for i in range(len(q_target)):
            errorCode = sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                       targetPosition=q_target[i],
                                                       operationMode=sim.simx_opmode_oneshot)
        if precision:
            self.wait_till_joint_position_is_met(q_target)
        else:
            self.wait()
        self.q_path.append(q_target)

    def set_joint_positions(self, q_target):
        """
        CAUTION: this function may only work if the "position control loop" is enabled at every arm joint.
        :param precision: whether to wait for Coppelia until que joint values are attained with precision
                    precision=True: --> the method self.wait_till_joint_position_is_met is called. This method
                    checks, at each simulation time, whether the specified joint values q_target have been achieved.
        :return: None
        """
        for i in range(len(q_target)):
            errorCode = sim.simxSetJointPosition(clientID=self.clientID,
                                                 jointHandle=self.armjoints[i],
                                                 position=q_target[i],
                                                 operationMode=sim.simx_opmode_oneshot_wait)
        # self.q_path.append(q_target)

    def set_joint_target_trajectory(self, q_path, sampling=1, precision='last'):
        """
        A repeated call to set_joint_target_positions.
        param q: a list of qs (joint positions).
        param sampling: select sampling=1 to reproduce all the joint positions in the path
                  select sampling=2 to skip one out of two joint positions.
        param precision:
        all: wait for the simulator to wait until every q in the path is attained with precision
                    (wait_till_joint_position_is_met) is called for every index i in  the trajectory.
        last:  wait for the simulator to wait only on the last joint values q in  the trajectory.
                   (wait_till_joint_position_is_met) is called on the last i in  the trajectory.
        low: a low precision over the trajectory.
        none:
                    if wait=True, the robot control scheme is making the joints stop and start
                    for each new joint. It waits until the joints of the robot and the target are equal.
                    With this option, the movement of the robot may be non-smooth.
                    if wait=False: the movement is typically smoother, but the trajectory is not followed exaclty.
        """
        # precision must be attained on all movements
        if precision == 'all' or precision is True:
            samples = range(0, len(q_path), sampling)
            for i in samples:
                self.set_joint_target_positions(q_path[i], precision=True)
        # precision is low on any
        elif precision == 'low':
            self.epsilonq = 1000.0 * self.epsilonq
            for i in range(0, len(q_path), sampling):
                self.set_joint_target_positions(q_path[i], precision=True)
            self.epsilonq = self.epsilonq / 1000.0
        # precision must be attained only on the last i in the path
        elif precision == 'last':
            samples = range(0, len(q_path), sampling)
            for i in samples[0:-1]:
                self.set_joint_target_positions(q_path[i], precision=False)
            self.set_joint_target_positions(q_path[-1], precision=True)
        # precision must not be attained on any i in the path
        elif precision == 'none' or precision is False:
            samples = range(0, len(q_path), sampling)
            for i in samples:
                self.set_joint_target_positions(q_path[i], precision=False)

    def get_joint_positions(self):
        q_actual = np.zeros(len(self.armjoints))
        n = len(self.armjoints)
        for i in range(0, n):
            while True:
                error, value = sim.simxGetJointPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                        operationMode=sim.simx_opmode_oneshot_wait)
                if error == 0:
                    q_actual[i] = value
                    break
        return q_actual

    def get_end_effector_position_orientation(self):
        errorCode, position = sim.simxGetObjectPosition(self.clientID, self.end_effector, -1,
                                                        sim.simx_opmode_oneshot_wait)
        errorCode, orientation = sim.simxGetObjectOrientation(self.clientID, self.end_effector, -1,
                                                        sim.simx_opmode_oneshot_wait)
        return position, orientation

    def get_target_position_orientation(self):
        errorCode, position = sim.simxGetObjectPosition(self.clientID, self.target, -1,
                                                        sim.simx_opmode_oneshot_wait)
        errorCode, orientation = sim.simxGetObjectOrientation(self.clientID, self.target, -1,
                                                        sim.simx_opmode_oneshot_wait)
        return position, orientation

    def set_target_position_orientation(self, position, orientation):
        errorCode = sim.simxSetObjectPosition(clientID=self.clientID, objectHandle=self.target,
                                              relativeToObjectHandle=-1, position=position,
                                              operationMode=sim.simx_opmode_oneshot_wait)
        errorCode = sim.simxSetObjectOrientation(clientID=self.clientID, objectHandle=self.target,
                                                 eulerAngles=orientation, relativeToObjectHandle=-1,
                                                 operationMode=sim.simx_opmode_oneshot_wait)
        return position, orientation

    def get_min_distance_to_objects(self):
        """
        Caution: a signal must have been added to the Coppelia Simulation (called distance_to_sphere)
        """
        error, distance = sim.simxGetFloatSignal(self.clientID, 'min_distance_to_objects', sim.simx_opmode_oneshot_wait)
        return distance

    def get_laser_data(self):
        """
        This reads the laserdata signal in Coppelia and returns it.
        The laserdata signal must be defined as in the UR5_velodyne.ttt environment.
        """
        error, data = sim.simxGetStringSignal(self.clientID, 'laserdata', sim.simx_opmode_oneshot_wait)
        # TODO: after unpacking the floats, some more-readable data structure should be built.
        data = sim.simxUnpackFloats(data)
        return data

    def stop_arm(self):
        for armj in self.armjoints:
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=armj,
                                                       targetVelocity=0.0, operationMode=sim.simx_opmode_oneshot)

    def stop_simulation(self):
        sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(self.clientID)

    def wait(self, steps=1):
        for i in range(0, steps):
            sim.simxSynchronousTrigger(clientID=self.clientID)

    def wait_till_joint_position_is_met(self, q_target):
        n_iterations = 0
        while True:
            # make the simulation go forward 1 step
            sim.simxSynchronousTrigger(clientID=self.clientID)
            q_actual = self.get_joint_positions()
            error = np.linalg.norm(q_target-q_actual)
            # print('Current error is:', error)
            # print('n_iterations: ', n_iterations)
            if error < self.epsilonq:
                break
            if n_iterations > self.max_iterations_joint_target:
                print('ERROR, joint position could not be achieved, try increasing max_iterations')
                print('Errors (q)')
                print(q_target-q_actual)
                print('Total error is:', np.linalg.norm(q_target-q_actual))
                break
            n_iterations += 1

    def get_jacobian(self, q):
        # calling derived class get_jacobian
        # should be implemented at the UR5, UR10 classes etc.
        return self.get_jacobian(q)

    def compute_manipulability(self, q):
        [J, _, _] = self.get_jacobian(q)
        manip = np.sqrt(np.linalg.det(np.dot(J, J.T)))
        return manip

    def directkinematics(self, q):
        T = self.serialrobot.directkinematics(q)
        return T # self.direct_kinematics(q)

    def compute_target_error(self, targetposition, targetorientation):
        """
        computes a euclidean distance in px, py, pz between target position and the robot's end effector
        computes a orientation error based on the quaternion orientation vectors
        """
        position, orientation = self.get_end_effector_position_orientation()
        error_dist = np.array(targetposition)-np.array(position)
        # transform to rotation matrix and then to quaternion.
        # please, beware that the orientation is not unique using alpha, beta, gamma
        Rorientation = euler2rot(orientation)
        Rtargetorientation = euler2rot(targetorientation)
        Qorientation = rot2quaternion(Rorientation)
        Qtargetorientation = rot2quaternion(Rtargetorientation)
        error_orient = Qorientation[1:4]-Qtargetorientation[1:4]
        return np.linalg.norm(error_dist), np.linalg.norm(error_orient)


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

    def inversekinematics(self, target_position, target_orientation, q0):
        """
        Solve the inverse kinematics using a Jacobian method.
        target_position: XYX vector in global coordinates.
        target_orientation: A quaternion specifying orientation.
        """
        # build transform using position and Quaternion
        Ttarget = HomogeneousMatrix(target_position, target_orientation)
        q = q0
        for i in range(0, self.max_iterations_inverse_kinematics):
            print('Iteration number: ', i)
            Ti = self.directkinematics(q)
            e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
            print('errordist, error orient: ', error_dist, error_orient)
            if error_dist < self.max_error_dist_inversekinematics and error_orient < self.max_error_orient_inversekinematics:
                print('Converged!!')
                break
            J, Jv, Jw = self.get_jacobian(q)
            qd = delta_q(J, e, method=self.ikmethod)
            q = q + qd
            if self.do_apply_joint_limits:
                [q, _] = self.apply_joint_limits(q)
        return q

    def inversekinematics_line(self, target_position, target_orientation, q0, vmax=1.0, sphere_position=None):
        """
        The end effector should follow a line in task space to reach target position and target orientation.
        A number of points is interpolated along the line, according to the speed vmax and simulation time
        (delta_time).
        The same number or points are also interpolated in orientation.
        Caution. target_orientationQ is specified as a quaternion
        """
        Ttarget = HomogeneousMatrix(target_position, target_orientation)
        Ti = self.directkinematics(q0)
        Qcurrent = Ti.Q()
        Qtarget = target_orientation.Q()
        p_current = Ti.pos()
        p_target = Ttarget.pos()
        n = n_movements(p_current, p_target, vmax)
        # generate n target positions
        target_positions = generate_target_positions(p_current, p_target, n)
        # generating quaternions on the line. Use SLERP to interpolate between quaternions
        target_orientations = generate_target_orientations_Q(Qcurrent, Qtarget, len(target_positions))
        if sphere_position is not None:
            target_positions = move_target_positions_obstacles(target_positions, sphere_position)
            # p_positions = np.array(target_positions)
            # plot3d(p_positions[:, 0], p_positions[:, 1], p_positions[:, 2])
        q_path = []
        q = q0
        # now try to reach each target position on the line
        for i in range(len(target_positions)):
            q = self.inversekinematics(target_position=target_positions[i],
                                       target_orientation=target_orientations[i], q0=q)
            q_path.append(q)
        return q_path

    def inversekinematics_path(self, target_positions, target_orientations, q0):
        """
        Solve iteratively q for each of the target positions and orientation specified
        """
        q_path = []
        q = q0
        # now try to reach each target position on the line
        for i in range(len(target_positions)):
            q = self.inversekinematics(target_position=target_positions[i],
                                       target_orientation=target_orientations[i], q0=q)
            q_path.append(q)
        return q_path

    def get_image(self):
        print('Capturing image of vision sensor ')
        # define here the fov of the camera
        fov = 60 # degrees
        # change fov to rad
        fov = fov * np.pi / 180.0
        sim.simxSetObjectFloatParameter(self.clientID, self.camera,
                                        sim.sim_visionfloatparam_perspective_angle,
                                        fov,
                                        sim.simx_opmode_oneshot_wait)
        # Get the image of vision sensor.
        # to ensure that the image is received, first streaming, then buffer
        errorCode, resolution, image = sim.simxGetVisionSensorImage(self.clientID, self.camera, 0,
                                                                    sim.simx_opmode_streaming)
        time.sleep(0.5)
        errorCode, resolution, image = sim.simxGetVisionSensorImage(self.clientID, self.camera, 0,
                                                                    sim.simx_opmode_buffer)
        print('Image captured!')
        return image, resolution

    def save_image(self, image, resolution, filename):
        sensorImage = np.array(image, dtype=np.uint8)
        sensorImage.resize([resolution[1], resolution[0], 3])
        img = Image.fromarray(sensorImage)
        img = ImageOps.flip(img)
        print('Saving to file: ', filename)
        img.save(filename)
        # caution--> tell python to release memory now or a kill will be issued by the system!
        del img
        del sensorImage
        print('Image saved!')

    def plot_trajectories(self):
        plt.figure()
        q_path = np.array(self.q_path)
        if len(q_path) == 0:
            return
        sh = q_path.shape
        for i in range(0, sh[1]):
            plt.plot(q_path[:, i], label='q' + str(i + 1))
        plt.legend()
        plt.title('JOINT TRAJECTORIES')
        plt.show(block=True)

    def get_trajectories(self):
        return self.q_path

