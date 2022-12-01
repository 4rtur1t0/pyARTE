#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/youbot.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021

"""
import time
import sim
import sys
import numpy as np
# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


# def rot2eulXYZ(R):
#     """
#     Computes Euler angles for the expression Rx(alpha)Ry(beta)Rz(gamma)
#     :param R:
#     :return:
#     """
#     # caution, c-like indexes in python!
#     sbeta = R[0, 2]
#     if abs(sbeta) == 1.0:
#         # degenerate case in which sin(beta)=+-1 and cos(beta)=0
#         # arbitrarily set alpha to zero
#         alpha = 0.0
#         beta = np.arcsin(sbeta)
#         gamma = np.arctan2(R[1, 1], R[1, 0])
#     else:
#         # standard way to compute alpha beta and gamma
#         alpha = -np.arctan2(R[1, 2], R[2, 2])
#         beta = np.arctan2(np.cos(alpha) * R[0, 2], R[2, 2])
#         gamma = -np.arctan2(R[0, 1], R[0, 0])
#     return [alpha, beta, gamma]

class Robot():
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper):
        self.clientID = clientID
        self.wheeljoints = wheeljoints
        self.armjoints = armjoints
        self.base = base
        self.gripper = gripper
        self.joint_directions = [1, -1, -1, -1, 1]

    def set_base_velocity(self, forwBackVel, leftRightVel, rotVel):
        rotVel=-rotVel
        # leftRightVel = leftRightVel
        errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID,
                                                   jointHandle=self.wheeljoints[0],
                                                   targetVelocity=-forwBackVel-leftRightVel-rotVel,
                                                   operationMode=sim.simx_opmode_oneshot)
        errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID,
                                                   jointHandle=self.wheeljoints[1],
                                                   targetVelocity=-forwBackVel+leftRightVel-rotVel,
                                                   operationMode=sim.simx_opmode_oneshot)
        errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID,
                                                   jointHandle=self.wheeljoints[2],
                                                   targetVelocity=-forwBackVel-leftRightVel+rotVel,
                                                   operationMode=sim.simx_opmode_oneshot)
        errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID,
                                                   jointHandle=self.wheeljoints[3],
                                                   targetVelocity=-forwBackVel+leftRightVel+rotVel,
                                                   operationMode=sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def set_base_position_orientation(self, position, phi):
        position = np.array(position)
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        Rz = np.array([[cphi, -sphi, 0], [sphi, cphi, 0], [0, 0, 1]])
        # alpha_beta_gamma = rot2eulXYZ(Rz)
        alpha_beta_gamma = [0, 0, 0.78]
        errorCode = sim.simxSetObjectPosition(self.clientID, self.base, -1, position, sim.simx_opmode_oneshot_wait)
        errorCode = sim.simxSetObjectOrientation(self.clientID, self.base, sim.sim_handle_parent, alpha_beta_gamma, sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def set_arm_velocity(self, qd):
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
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def set_arm_joint_position(self, q):
        """
        CAUTION: this function may only work if the "position control loop" is enabled at every youbot armjoint.
        :param q:
        :return:
        """
        for i in range(0, len(q)):
            errorCode = sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
                                                       targetPosition=self.joint_directions[i]*q[i],
                                                       operationMode=sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def stop_base(self):
        for wheelj in self.wheeljoints:
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=wheelj,
                                                       targetVelocity=0.0, operationMode=sim.simx_opmode_oneshot)
            sim.simxSynchronousTrigger(clientID=self.clientID)

    def stop_arm(self):
        for armj in self.armjoints:
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=armj,
                                                       targetVelocity=0.0, operationMode=sim.simx_opmode_oneshot)
            sim.simxSynchronousTrigger(clientID=self.clientID)

    def open_gripper(self):
        # sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.gripper[1],
        #                                targetVelocity=-0.04, operationMode=sim.simx_opmode_oneshot)
        # j2 = sim.simxGetJointPosition(clientID=self.clientID, jointHandle=self.gripper[1],
        #                               operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[0],
                                       targetPosition=-0.05, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[1],
                                       targetPosition=-0.05, operationMode=sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def close_gripper(self):
        # sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.gripper[1],
        #                                targetVelocity=0.04, operationMode=sim.simx_opmode_oneshot)
        # j2 = sim.simxGetJointPosition(clientID=self.clientID, jointHandle=self.gripper[1],
        #                               operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[0],
                                       targetPosition=0.0, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[1],
                                       targetPosition=0.0, operationMode=sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID=self.clientID)


    def wait(self, steps):
        for i in range(0, steps):
            sim.simxSynchronousTrigger(clientID=self.clientID)

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


def main_loop():
    # global pathpositions
    # Python connect to the V-REP client
    sim.simxFinish(-1)

    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print("Connected to remote API server")
        # stop previous simiulation
        sim.simxStopSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
        time.sleep(1)
        sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
        # enable the synchronous mode
        sim.simxSynchronous(clientID=clientID, enable=True)
        #time.sleep(3)
    else:
        print("Connection not successful")
        sys.exit("Connection failed,program ended!")

    wheeljoints = []
    armjoints = []
    gripper = []
    objects = []
    # Get the handles of the relevant objects
    errorCode, youbotbase = sim.simxGetObjectHandle(clientID, 'youBot', sim.simx_opmode_oneshot_wait)
    errorCode, fl = sim.simxGetObjectHandle(clientID, 'rollingJoint_fl', sim.simx_opmode_oneshot_wait)
    errorCode, rl = sim.simxGetObjectHandle(clientID, 'rollingJoint_rl', sim.simx_opmode_oneshot_wait)
    errorCode, rr = sim.simxGetObjectHandle(clientID, 'rollingJoint_rr', sim.simx_opmode_oneshot_wait)
    errorCode, fr = sim.simxGetObjectHandle(clientID, 'rollingJoint_fr', sim.simx_opmode_oneshot_wait)
    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'youBotArmJoint0', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'youBotArmJoint1', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'youBotArmJoint2', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'youBotArmJoint3', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'youBotArmJoint4', sim.simx_opmode_oneshot_wait)
    errorCode, gripper1 = sim.simxGetObjectHandle(clientID, 'youBotGripperJoint1', sim.simx_opmode_oneshot_wait)
    errorCode, gripper2 = sim.simxGetObjectHandle(clientID, 'youBotGripperJoint2', sim.simx_opmode_oneshot_wait)
    errorCode, sphere = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_oneshot_wait)

    wheeljoints.append(fl)
    wheeljoints.append(rl)
    wheeljoints.append(rr)
    wheeljoints.append(fr)
    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    gripper.append(gripper1)
    gripper.append(gripper2)

    objects.append(sphere)

    robot = Robot(clientID=clientID, wheeljoints=wheeljoints,
                  armjoints=armjoints, base=youbotbase, gripper=gripper)
    scene = Scene(clientID=clientID, objects=objects)

    # robot.set_base_position_orientation([0, 0, +9.5750e-02], phi=0)

    for i in range(0, 150):
        robot.set_base_velocity(1.0, 0.0, 0)

    for i in range(0, 150):
        robot.set_base_velocity(0, 1.0, 0)
    #
    # for i in range(0, 150):
    #     robot.set_base_velocity(0, 0, 1.0)

    # q = [0.8, 0.8, 0.8, 0.8, 0.8]
    # qd = [0.1, 0.1, 0.1, 0.1, 0.1]
    # for i in range(0, 150):
    #     robot.set_arm_velocity(qd)

    # robot.set_arm_joint_position(q)
    # robot.wait(50)
    robot.close_gripper()
    # robot.wait(50)
    # robot.open_gripper()
    # robot.wait(50)




    # for i in range(0, 50):
    #     scene.random_walk()

    # stop everything
    robot.stop_base()
    robot.stop_arm()




if __name__ == "__main__":
    main_loop()
