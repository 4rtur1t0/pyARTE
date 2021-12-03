#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021

"""
import time
import sim
import sys
import numpy as np
# standard delta time for Coppelia, please modify if necessary
from kinematics.kinematics_ur10 import eval_symbolic_jacobian_UR10, eval_symbolic_jacobian_lbr_kuka
import matplotlib.pyplot as plt


DELTA_TIME = 50.0/1000.0


def euler2Rot(abg):
    calpha = np.cos(abg[0])
    salpha = np.sin(abg[0])
    cbeta = np.cos(abg[1])
    sbeta = np.sin(abg[1])
    cgamma = np.cos(abg[2])
    sgamma = np.sin(abg[2])
    Rx = np.array([[1, 0, 0], [0, calpha, -salpha], [0, salpha, calpha]])
    Ry = np.array([[cbeta, 0, sbeta], [0, 1, 0], [-sbeta, 0, cbeta]])
    Rz = np.array([[cgamma, -sgamma, 0], [sgamma, cgamma, 0], [0, 0, 1]])

    R = np.matmul(Rx, Ry)
    R = np.matmul(R, Rz)
    return R

def rot2eulXYZ(R):
    """
    Computes Euler angles for the expression Rx(alpha)Ry(beta)Rz(gamma)
    :param R:
    :return:
    """
    # caution, c-like indexes in python!
    sbeta = R[0, 2]
    if abs(sbeta) == 1.0:
        # degenerate case in which sin(beta)=+-1 and cos(beta)=0
        # arbitrarily set alpha to zero
        alpha = 0.0
        beta = np.arcsin(sbeta)
        gamma = np.arctan2(R[1, 1], R[1, 0])
    else:
        # standard way to compute alpha beta and gamma
        alpha = -np.arctan2(R[1, 2], R[2, 2])
        beta = np.arctan2(np.cos(alpha) * R[0, 2], R[2, 2])
        gamma = -np.arctan2(R[0, 1], R[0, 0])
    return [alpha, beta, gamma]


class RobotLBRKUKA():
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target):
        self.clientID = clientID
        self.wheeljoints = wheeljoints
        self.armjoints = armjoints
        self.base = base
        self.gripper = gripper
        self.end_effector = end_effector
        self.joint_directions = [1, 1, 1, 1, 1, 1, 1]
        self.epsilonq = 0.0005
        self.max_iterations = 500
        self.target = target
        self.maxjspeeds = np.array([110, 110, 128, 128, 204, 184, 184])
        self.maxjspeeds = self.maxjspeeds*np.pi/180.0

    def set_arm_joint_target_velocities(self, qd):
        """
        CAUTION: this function does only work if the position control loop is disabled at every youbot armjoint.
        Set the arm joint speeds
        :param qd: joint speeds rad/s
        :return:
        """
        for i in range(0, len(qd)):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                       targetVelocity=self.joint_directions[i]*qd[i],
                                                       operationMode=sim.simx_opmode_oneshot)

    def set_arm_joint_target_positions(self, q_target):
        """
        CAUTION: this function may only work if the "position control loop" is enabled at every youbot armjoint.
        :param q:
        :return:
        """
        for i in range(0, len(q_target)):
            errorCode = sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                       targetPosition=self.joint_directions[i]*q_target[i],
                                                       operationMode=sim.simx_opmode_oneshot)

    def set_arm_joint_positions(self, q_target):
        """
        CAUTION: this function may only work if the "position control loop" is enabled at every youbot armjoint.
        :param q:
        :return:
        """
        for i in range(0, len(q_target)):
            errorCode = sim.simxSetJointPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                 position=self.joint_directions[i]*q_target[i],
                                                 operationMode=sim.simx_opmode_oneshot)

    def open_gripper(self):
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[0],
                                       targetPosition=0.0, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[1],
                                       targetPosition=0.0, operationMode=sim.simx_opmode_oneshot)

    def close_gripper(self):
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[0],
                                       targetPosition=-0.2, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[1],
                                       targetPosition=-0.2, operationMode=sim.simx_opmode_oneshot)

    def wait_till_joint_position_is_met(self, q_target):
        n_iterations = 0
        while True:
            # make the simulation go forward 1 step
            sim.simxSynchronousTrigger(clientID=self.clientID)
            q_actual = self.get_arm_joint_positions()
            error = np.linalg.norm(q_target-q_actual)
            # print('Current error is:', error)
            # print('n_iterations: ', n_iterations)
            if error < self.epsilonq:
                break
            if n_iterations > self.max_iterations:
                print('ERROR, joint position could not be achieved, try increasing max_iterations')
                break
            n_iterations += 1

    def get_arm_joint_positions(self):
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

    def stop_arm(self):
        for armj in self.armjoints:
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=armj,
                                                       targetVelocity=0.0, operationMode=sim.simx_opmode_oneshot)

    def wait(self, steps):
        for i in range(0, steps):
            sim.simxSynchronousTrigger(clientID=self.clientID)

    def compute_vref_wref(self):
        position, orientation = self.get_end_effector_position_orientation()
        targetposition, targetorientation = self.get_target_position_orientation()
        vref = np.array(targetposition)-np.array(position)
        n = np.linalg.norm(vref)
        vref = vref/n
        wref = np.array([0, 0, 0])
        vwref = np.hstack((vref, wref))
        return vwref

    def compute_target_error(self):
        position, orientation = self.get_end_effector_position_orientation()
        targetposition, targetorientation = self.get_target_position_orientation()
        error = np.array(targetposition)-np.array(position)
        return np.linalg.norm(error)

    # def saturate_qd(self, qd):
    #     diffs = np.zeros(len(qd))
    #     for i in range(0, len(qd)):
    #         diffs[i] = np.abs(qd[i])-np.abs(self.maxjspeeds[i])
    #
    #     mx = np.max(diffs)



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


def init_simulation():
    # global pathpositions
    # Python connect to the V-REP client
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print("Connected to remote API server")
        # stop previous simiulation
        sim.simxStopSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
        time.sleep(3)
        sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
        # enable the synchronous mode
        sim.simxSynchronous(clientID=clientID, enable=True)
        # time.sleep(3)
    else:
        print("Connection not successful")
        sys.exit("Connection failed,program ended!")

    armjoints = []
    gripper = []
    objects = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint6', sim.simx_opmode_oneshot_wait)
    errorCode, q7 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint7', sim.simx_opmode_oneshot_wait)

    errorCode, gripper1 = sim.simxGetObjectHandle(clientID, 'Barrett_openCloseJoint', sim.simx_opmode_oneshot_wait)
    errorCode, gripper2 = sim.simxGetObjectHandle(clientID, 'Barrett_openCloseJoint0', sim.simx_opmode_oneshot_wait)

    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)

    # errorCode, sphere = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_oneshot_wait)

    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    armjoints.append(q7)
    gripper.append(gripper1)
    gripper.append(gripper2)

    # objects.append(sphere)
    robot = RobotLBRKUKA(clientID=clientID, wheeljoints=[],
                        armjoints=armjoints, base=robotbase,
                        end_effector=end_effector, gripper=gripper, target=target)
    scene = Scene(clientID=clientID, objects=objects)
    return robot, scene


def plot_trajectories(q_rs):
    q_rs = np.array(q_rs)
    plt.figure()

    for i in range(0, 7):
        plt.plot(q_rs[:, i], label='q' + str(i + 1))
    plt.legend()
    plt.show(block=True)

def moore_penrose(J):
    sq = np.dot(J.T, J)
    print('manip is: ', np.linalg.det(sq))
    sq = np.linalg.inv(sq)
    return np.dot(sq, J.T)



def main_loop():
    robot, scene = init_simulation()

    q = np.array([1, 1, 1, 1, 1, 1, 1])
    # q = np.dot(np.pi/2, q)
    q = np.dot(0.4, q)
    robot.set_arm_joint_target_positions(q)
    robot.wait_till_joint_position_is_met(q)

    # position, orientation = robot.get_end_effector_position_orientation()
    # R = euler2Rot(orientation)
    # print('position:', position)
    # print(orientation)
    # print(R)
    #
    vmag = 0.3
    q_rs = []
    # get current joint values
    for i in range(0, 40):
        vref = robot.compute_vref_wref()
        error = robot.compute_target_error()
        print('error: ', error)
        if error < 0.01:
            break
        vref = vmag*vref
        # get actual coordinates of the arm
        q = robot.get_arm_joint_positions()
        J = eval_symbolic_jacobian_lbr_kuka(q)
        print('Manip is: ', np.linalg.det(np.dot(J, J.T)))
        # moore penrose
        iJ = np.linalg.pinv(J)
        # iJ2 = moore_penrose(J)
        qd = np.dot(iJ, vref.T)
        print('Joint speed norm: ', np.linalg.norm(qd))
        vrefreal = np.dot(J, qd)
        # print('J: ', J)
        # print('iJ: ', iJ2)
        qd = np.dot(DELTA_TIME, qd)
        # qd = robot.saturate_qd(qd)
        # increase coordinates
        qnew = q + qd
        q_rs.append(qnew)
        robot.set_arm_joint_target_positions(qnew)
        robot.wait_till_joint_position_is_met(qnew)
        # robot.wait(1)

    plot_trajectories(q_rs)


    robot.stop_arm()
    scene.stop_simulation()


if __name__ == "__main__":
    # pid_loop()
    # speed_loop()
    main_loop()
