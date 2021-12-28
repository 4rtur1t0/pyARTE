#!/usr/bin/env python
# encoding: utf-8
"""
Base Robot Class

@Authors: Arturo Gil
@Time: April 2021

"""
import sim
import numpy as np
from artelib.tools import compute_w_between_orientations, euler2Rot, R2quaternion, buildT, compute_w_between_R
import matplotlib.pyplot as plt
import time
from PIL import Image, ImageOps


DELTA_TIME = 50.0/1000.0


class Robot():
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target, camera,
                 max_joint_speeds, joint_ranges):
        self.clientID = clientID
        self.wheeljoints = wheeljoints
        self.armjoints = armjoints
        self.base = base
        self.gripper = gripper
        self.end_effector = end_effector
        self.camera = camera
        # self.max_iterations = 1500
        self.target = target
        self.max_joint_speeds = max_joint_speeds
        self.joint_ranges = joint_ranges
        # parameters of the inverse kinematics algorith
        self.max_iterations_inverse_kinematics = 1500
        self.max_error_dist_inversekinematics = 0.010
        self.max_error_orient_inversekinematics = 0.010
        # max iterations to achieve a joint target in coppelia
        self.max_iterations_joint_target = 200
        # admit this error in q
        self.epsilonq = 0.0005
        self.q_path = []

    def set_joint_target_velocities(self, qd):
        """
        CAUTION: this function does only work if the position control loop is disabled at every youbot armjoint.
        Set the arm joint speeds
        :param qd: joint speeds rad/s
        :return:
        """
        for i in range(0, len(qd)):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                       targetVelocity=qd[i],
                                                       operationMode=sim.simx_opmode_oneshot)

    def set_joint_target_positions(self, q_target, wait=False):
        """
        CAUTION: this function may only work if the "position control loop" is enabled at every youbot armjoint.
        :param q:
        :return:
        """
        for i in range(0, len(q_target)):
            errorCode = sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                       targetPosition=q_target[i],
                                                       operationMode=sim.simx_opmode_oneshot)
        if wait:
            self.wait_till_joint_position_is_met(q_target)
        self.q_path.append(q_target)

    def set_joint_target_trajectory(self, q_path, sampling=1, wait=True):
        """
        A repeated call to set_joint_target_positions.
        param q: a list of qs (joint positions).
        param sampling: select sampling=1 to reproduce all the joint positions in the path
                  select sampling=2 to skip one out of two joint positions.
        param wait: whether wait_till_joint_position_is met should be called for each joint position in the path.
                    if wait=True, the robot control scheme is making the joints stop and start
                    for each new joint. It waits until the joints of the robot and the target are equal.
                    With this option, the movement of the robot may be non-smooth.
                    if wait=False: the movement is typically smoother, but the trajectory is not followed exaclty.
        """
        samples = range(0, len(q_path), sampling)
        for i in samples:
            self.set_joint_target_positions(q_path[i])
            if wait:
                self.wait_till_joint_position_is_met(q_path[i])
            else:
                self.wait()
            self.q_path.append(q_path[i])

    def get_joint_positions(self):
        q_actual = np.zeros(len(self.armjoints))
        n = len(self.armjoints)
        for i in range(0, n):
            while True:
                error, value = sim.simxGetJointPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                        operationMode=sim.simx_opmode_oneshot)
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

    def stop_arm(self):
        for armj in self.armjoints:
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=armj,
                                                       targetVelocity=0.0, operationMode=sim.simx_opmode_oneshot)

    def wait(self, steps=1):
        for i in range(0, steps):
            sim.simxSynchronousTrigger(clientID=self.clientID)

    def get_jacobian(self, q):
        # calling derived class get_jacobian
        # should be implemented at the UR5, UR10 classes etc.
        return self.get_jacobian(q)

    def compute_manipulability(self, q):
        [J, _, _] = self.get_jacobian(q)
        manip = np.sqrt(np.linalg.det(np.dot(J, J.T)))
        return manip

    def direct_kinematics(self, q):
        return self.direct_kinematics(q)

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
                break
            n_iterations += 1

    def compute_vref_wref(self, targetposition, targetorientation):
        position, orientation = self.get_end_effector_position_orientation()
        vref = np.array(targetposition)-np.array(position)
        wref = compute_w_between_orientations(orientation, targetorientation)
        vref = vref/np.linalg.norm(vref)
        wref = wref/np.linalg.norm(wref)
        vwref = np.hstack((vref, wref))
        return vwref, vref, wref

    def compute_target_error(self, targetposition, targetorientation):
        """
        computes a euclidean distance in px, py, pz between target position and the robot's end effector
        computes a orientation error based on the quaternion orientation vectors
        """
        position, orientation = self.get_end_effector_position_orientation()
        error_dist = np.array(targetposition)-np.array(position)
        # transform to rotation matrix and then to quaternion.
        # please, beware that the orientation is not unique using alpha, beta, gamma
        Rorientation = euler2Rot(orientation)
        Rtargetorientation = euler2Rot(targetorientation)
        Qorientation = R2quaternion(Rorientation)
        Qtargetorientation = R2quaternion(Rtargetorientation)
        error_orient = Qorientation[1:4]-Qtargetorientation[1:4]
        return np.linalg.norm(error_dist), np.linalg.norm(error_orient)

    def compute_actions(self, Tcurrent, Ttarget, vmax=1):
        """
        Compute the movement that allows to bring Tcurrent to Ttarget with a given linear max speed
        """
        vref = np.array(Ttarget[0:3, 3])-np.array(Tcurrent[0:3, 3])
        wref = compute_w_between_R(Tcurrent, Ttarget)
        # Compute error in distance and error in orientation.
        # The error in orientation is computed
        error_dist = np.linalg.norm(vref)
        error_orient = np.linalg.norm(wref)
        vwref = np.hstack((vref, wref))
        return vwref, error_dist, error_orient

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
            if (self.joint_ranges[0, i] < q[i]) and (self.joint_ranges[1, i] > q[i]):
                valid_indexes.append(True)
                continue
            else:
                print(30*'*')
                print('JOINT ERROR: RANGE ERROR! Joint: q', i+1, ' is out of range')
                print(30 * '*')
                valid = False
                valid_indexes.append(False)
        return valid, valid_indexes

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
        print('Joint speed norm: ', np.linalg.norm(qd))
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

    def moore_penrose_damped(self, J, vwref):
        """
        Considers a simple joint control to behave properly in the presence of a singularity
        """
        manip = np.linalg.det(np.dot(J, J.T))
        print('Manip is: ', manip)
        # print('Manip v is: ', np.linalg.det(np.dot(Jv, Jv.T)))
        # print('Manip w is: ', np.linalg.det(np.dot(Jw, Jw.T)))
        # normal case --> just compute pseudo inverse
        # we are far from a singularity
        if manip > .01 ** 2:
            # iJ = np.linalg.pinv(J)
            # moore penrose pseudo inverse J^T(J*J^T)^{-1}
            iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
            qd = np.dot(iJ, vwref.T)
            return qd
        print('Close to singularity: implementing DAMPED Least squares solution')
        K = 0.01 * np.eye(np.min(J.shape))
        iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + K))
        qd = np.dot(iJ, vwref.T)
        return qd

    def adjust_vwref(self, vwref, error_dist, error_orient):
        vmag = 0.5
        wmag = 0.8
        # ACTIVIDAD: vmag and wmag for a linear deceleration and better convergence
        if error_dist < .05:
            vmag = vmag * error_dist + 0.01
        if error_orient < .05:
            wmag = wmag * error_orient + 0.01
        # linear speed
        vref = vwref[0:3]
        # angular speed
        wref = vwref[3:6]
        nvref = np.linalg.norm(vref)
        nwref = np.linalg.norm(wref)
        if nvref > 0.01:
            vref = vmag*vref/nvref
        if nwref > 0.01:
            wref = wmag * wref / nwref
        vwref = np.hstack((vref, wref))
        return vwref

    def inversekinematics_line(self, target_position, target_orientation, q0, fine=True, vmax=1):
        """
        fine: whether to reach the target point with precision or not.
        vmax: linear velocity of the planner.
        """
        Ttarget = buildT(target_position, target_orientation)
        q_path = []
        qd_path = []
        q = q0
        for i in range(0, self.max_iterations_inverse_kinematics):
            print('Iteration number: ', i)
            Ti = self.direct_kinematics(q)
            vwref, error_dist, error_orient = self.compute_actions(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax)
            print(Ttarget-Ti)
            vwref = self.adjust_vwref(vwref=vwref, error_dist=error_dist, error_orient=error_orient)
            print('vwref: ', vwref)
            print('errordist, error orient: ', error_dist, error_orient)
            if (not fine) and error_dist < 5.0*self.max_error_dist_inversekinematics and error_orient < 5.0*self.max_error_orient_inversekinematics:
                print('Converged!! (NOT FINE)')
                break
            if error_dist < self.max_error_dist_inversekinematics and error_orient < self.max_error_orient_inversekinematics:
                print('Converged!!')
                break
            J, Jv, Jw = self.get_jacobian(q)
            # compute joint speed to achieve the reference
            qd = self.moore_penrose_damped(J, vwref)
            # check joint speed and correct if necessary
            [qd, _, _] = self.check_speed(qd)
            # integrate movement. Please check that Delta_time matches coppelia simulation time step
            qd = np.dot(DELTA_TIME, qd)
            q = q + qd
            # check joints ranges
            self.check_joints(q)
            # append to the computed joint path
            q_path.append(q)
            qd_path.append(qd)
        return q_path, qd_path

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
        # flatten self.q_path
        q_path = np.array(self.q_path)
        for i in range(0, 6):
            plt.plot(q_path[:, i], label='q' + str(i + 1))
        plt.legend()
        plt.show(block=True)


class Scene():
    def __init__(self, clientID, objects):
        self.clientID = clientID
        self.objects = objects
        self.angle = 2.5

    def random_walk(self):
        errorCode, position = sim.simxGetObjectPosition(self.clientID, self.objects[0], -1, sim.simx_opmode_oneshot_wait)
        v = np.array([np.cos(self.angle), np.sin(self.angle), 0])
        # position
        position = np.array(position) + 0.1*v
        self.angle = self.angle + 0.1*np.random.rand(1, 1)
        errorCode = sim.simxSetObjectPosition(self.clientID, self.objects[0], -1, position, sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def stop_simulation(self):
        sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(self.clientID)

