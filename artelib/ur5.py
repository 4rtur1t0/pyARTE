#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before using this class.

@Authors: Arturo Gil
@Time: April 2021

"""
import sim
import numpy as np
from artelib.tools import compute_w_between_orientations
from artelib.robot import Robot
# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


class RobotUR5(Robot):
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target):
        Robot.__init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target)
        self.maxjspeeds = np.array([110, 110, 128, 128, 204, 184, 184])
        self.maxjspeeds = self.maxjspeeds*np.pi/180.0

    # def set_arm_joint_target_velocities(self, qd):
    #     """
    #     CAUTION: this function does only work if the position control loop is disabled at every youbot armjoint.
    #     Set the arm joint speeds
    #     :param qd: joint speeds rad/s
    #     :return:
    #     """
    #     for i in range(0, len(qd)):
    #         errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.armjoints[i],
    #                                                    targetVelocity=self.joint_directions[i]*qd[i],
    #                                                    operationMode=sim.simx_opmode_oneshot)
    #
    # def set_arm_joint_target_positions(self, q_target):
    #     """
    #     CAUTION: this function may only work if the "position control loop" is enabled at every youbot armjoint.
    #     :param q:
    #     :return:
    #     """
    #     for i in range(0, len(q_target)):
    #         errorCode = sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
    #                                                    targetPosition=self.joint_directions[i]*q_target[i],
    #                                                    operationMode=sim.simx_opmode_oneshot)
    #
    # def set_arm_joint_positions(self, q_target):
    #     """
    #     CAUTION: this function may only work if the "position control loop" is enabled at every youbot armjoint.
    #     :param q:
    #     :return:
    #     """
    #     for i in range(0, len(q_target)):
    #         errorCode = sim.simxSetJointPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
    #                                              position=self.joint_directions[i]*q_target[i],
    #                                              operationMode=sim.simx_opmode_oneshot)
    #
    # def open_gripper(self):
    #     sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[0],
    #                                    targetPosition=0.0, operationMode=sim.simx_opmode_oneshot)
    #     sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[1],
    #                                    targetPosition=0.0, operationMode=sim.simx_opmode_oneshot)

    def close_gripper(self):
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[0],
                                       targetPosition=-0.2, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[1],
                                       targetPosition=-0.2, operationMode=sim.simx_opmode_oneshot)

    # def wait_till_joint_position_is_met(self, q_target):
    #     n_iterations = 0
    #     while True:
    #         # make the simulation go forward 1 step
    #         sim.simxSynchronousTrigger(clientID=self.clientID)
    #         q_actual = self.get_arm_joint_positions()
    #         error = np.linalg.norm(q_target-q_actual)
    #         # print('Current error is:', error)
    #         # print('n_iterations: ', n_iterations)
    #         if error < self.epsilonq:
    #             break
    #         if n_iterations > self.max_iterations:
    #             print('ERROR, joint position could not be achieved, try increasing max_iterations')
    #             break
    #         n_iterations += 1
    #
    # def get_arm_joint_positions(self):
    #     q_actual = np.zeros(len(self.armjoints))
    #     n = len(self.armjoints)
    #     for i in range(0, n):
    #         while True:
    #             error, value = sim.simxGetJointPosition(clientID=self.clientID, jointHandle=self.armjoints[i],
    #                                                     operationMode=sim.simx_opmode_oneshot)
    #             if error == 0:
    #                 q_actual[i] = value
    #                 break
    #     return q_actual
    #
    # def get_end_effector_position_orientation(self):
    #     errorCode, position = sim.simxGetObjectPosition(self.clientID, self.end_effector, -1,
    #                                                     sim.simx_opmode_oneshot_wait)
    #     errorCode, orientation = sim.simxGetObjectOrientation(self.clientID, self.end_effector, -1,
    #                                                     sim.simx_opmode_oneshot_wait)
    #     return position, orientation
    #
    # def get_target_position_orientation(self):
    #     errorCode, position = sim.simxGetObjectPosition(self.clientID, self.target, -1,
    #                                                     sim.simx_opmode_oneshot_wait)
    #     errorCode, orientation = sim.simxGetObjectOrientation(self.clientID, self.target, -1,
    #                                                     sim.simx_opmode_oneshot_wait)
    #     return position, orientation
    #
    # def set_target_position_orientation(self, position, orientation):
    #     errorCode, position = sim.simxSetObjectPosition(self.clientID, self.target, -1, position,
    #                                                     sim.simx_opmode_oneshot_wait)
    #     errorCode, orientation = sim.simxSetObjectOrientation(self.clientID, self.target, -1, orientation,
    #                                                     sim.simx_opmode_oneshot_wait)
    #     return position, orientation
    #
    # def stop_arm(self):
    #     for armj in self.armjoints:
    #         errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=armj,
    #                                                    targetVelocity=0.0, operationMode=sim.simx_opmode_oneshot)
    #
    # def wait(self, steps):
    #     for i in range(0, steps):
    #         sim.simxSynchronousTrigger(clientID=self.clientID)
    #
    # def compute_vref_wref(self, targetposition, targetorientation):
    #     position, orientation = self.get_end_effector_position_orientation()
    #     # targetposition, targetorientation = self.get_target_position_orientation()
    #     vref = np.array(targetposition)-np.array(position)
    #     # n = np.linalg.norm(vref)
    #     wref = compute_w_between_orientations(orientation, targetorientation)
    #     vref = vref/np.linalg.norm(vref)
    #     wref = wref/np.linalg.norm(wref)
    #     vwref = np.hstack((vref, wref))
    #     return vwref, vref, wref
    #
    # def compute_target_error(self):
    #     position, orientation = self.get_end_effector_position_orientation()
    #     targetposition, targetorientation = self.get_target_position_orientation()
    #     error_dist = np.array(targetposition)-np.array(position)
    #     error_orient = np.array(targetorientation) - np.array(orientation)
    #     return np.linalg.norm(error_dist), np.linalg.norm(error_orient)

    # def saturate_qd(self, qd):
    #     diffs = np.zeros(len(qd))
    #     for i in range(0, len(qd)):
    #         diffs[i] = np.abs(qd[i])-np.abs(self.maxjspeeds[i])
    #
    #     mx = np.max(diffs)



# class Scene():
#     def __init__(self, clientID, objects):
#         self.clientID = clientID
#         self.objects = objects
#         self.angle = 2.5
#
#     def random_walk(self):
#         errorCode, position = sim.simxGetObjectPosition(self.clientID, self.objects[0], -1, sim.simx_opmode_oneshot_wait)
#         v = np.array([np.cos(self.angle), np.sin(self.angle), 0])
#         # position
#         position = np.array(position) + 0.1*v
#         self.angle = self.angle + 0.1*np.random.rand(1, 1)
#         errorCode = sim.simxSetObjectPosition(self.clientID, self.objects[0], -1, position, sim.simx_opmode_oneshot_wait)
#         sim.simxSynchronousTrigger(clientID=self.clientID)
#
#     def stop_simulation(self):
#         sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
#         sim.simxFinish(self.clientID)

#
# def init_simulation():
#     # global pathpositions
#     # Python connect to the V-REP client
#     sim.simxFinish(-1)
#     clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
#
#     if clientID != -1:
#         print("Connected to remote API server")
#         # stop previous simiulation
#         sim.simxStopSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
#         time.sleep(3)
#         sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
#         # enable the synchronous mode
#         sim.simxSynchronous(clientID=clientID, enable=True)
#         # time.sleep(3)
#     else:
#         print("Connection not successful")
#         sys.exit("Connection failed,program ended!")
#
#     armjoints = []
#     gripper = []
#     objects = []
#     # Get the handles of the relevant objects
#     errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'UR5', sim.simx_opmode_oneshot_wait)
#     errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)
#
#     errorCode, q1 = sim.simxGetObjectHandle(clientID, 'UR5_joint1', sim.simx_opmode_oneshot_wait)
#     errorCode, q2 = sim.simxGetObjectHandle(clientID, 'UR5_joint2', sim.simx_opmode_oneshot_wait)
#     errorCode, q3 = sim.simxGetObjectHandle(clientID, 'UR5_joint3', sim.simx_opmode_oneshot_wait)
#     errorCode, q4 = sim.simxGetObjectHandle(clientID, 'UR5_joint4', sim.simx_opmode_oneshot_wait)
#     errorCode, q5 = sim.simxGetObjectHandle(clientID, 'UR5_joint5', sim.simx_opmode_oneshot_wait)
#     errorCode, q6 = sim.simxGetObjectHandle(clientID, 'UR5_joint6', sim.simx_opmode_oneshot_wait)
#
#     # errorCode, gripper1 = sim.simxGetObjectHandle(clientID, 'Barrett_openCloseJoint', sim.simx_opmode_oneshot_wait)
#     # errorCode, gripper2 = sim.simxGetObjectHandle(clientID, 'Barrett_openCloseJoint0', sim.simx_opmode_oneshot_wait)
#
#     errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)
#
#     # errorCode, sphere = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_oneshot_wait)
#
#     armjoints.append(q1)
#     armjoints.append(q2)
#     armjoints.append(q3)
#     armjoints.append(q4)
#     armjoints.append(q5)
#     armjoints.append(q6)
#
#     # gripper.append(gripper1)
#     # gripper.append(gripper2)
#
#     # objects.append(sphere)
#     robot = RobotUR5(clientID=clientID, wheeljoints=[],
#                         armjoints=armjoints, base=robotbase,
#                         end_effector=end_effector, gripper=gripper, target=target)
#     scene = Scene(clientID=clientID, objects=objects)
#     return robot, scene
#
#
# def plot_trajectories(q_rs):
#     q_rs = np.array(q_rs)
#     plt.figure()
#
#     for i in range(0, 6):
#         plt.plot(q_rs[:, i], label='q' + str(i + 1))
#     plt.legend()
#     plt.show(block=True)
#
# def moore_penrose(J):
#     sq = np.dot(J.T, J)
#     print('manip is: ', np.linalg.det(sq))
#     sq = np.linalg.inv(sq)
#     return np.dot(sq, J.T)
#
#
#
# def main_loop():
#     max_iterations = 1500
#     robot, scene = init_simulation()
#
#     q = np.array([1, 1, 1, 1, 1, 1])
#     q = np.dot(np.pi/8, q)
#     # q = np.dot(0.0, q)
#     robot.set_arm_joint_target_positions(q)
#     robot.wait_till_joint_position_is_met(q)
#
#     # position, orientation = robot.get_end_effector_position_orientation()
#     # R = euler2Rot(orientation)
#     # print('position:', position)
#     # print('alphabetagamma: ', orientation)
#     # print('Rotation matrix R: ', R)
#
#     target_position = [0.3, 0.3, 0.5]
#     target_orientation = [0.0, 0.0, 0.0]
#     robot.set_target_position_orientation(target_position, target_orientation)
#     # vmag = 0.3
#     q_rs = []
#     for i in range(0, max_iterations):
#         print('Iteration number: ', i)
#         vwref, vref, wref = robot.compute_vref_wref(target_position, target_orientation)
#         error_dist, error_orient = robot.compute_target_error(target_position, target_orientation)
#         # TODO: ADJUST vmag for finer convergence
#         # ACTIVIDAD:
#         if error_dist > .1:
#             vmag = 0.3
#         else:
#             vmag = 3*error_dist + 0.01
#         if error_orient > .1:
#             wmag = 0.8
#         else:
#             wmag = 8 * error_orient + 0.01
#         vref = vmag*vref
#         wref = wmag*wref
#         vwref = np.hstack((vref, wref))
#         print('errordist, error orient: ', error_dist, error_orient)
#         if error_dist + error_orient < 0.01:
#             break
#         # get current coordinates of the arm
#         q = robot.get_arm_joint_positions()
#         J, Jv, Jw = eval_symbolic_jacobian_UR5(q)
#         print('Manip is: ', np.linalg.det(np.dot(J, J.T)))
#         print('Manip v is: ', np.linalg.det(np.dot(Jv, Jv.T)))
#         print('Manip w is: ', np.linalg.det(np.dot(Jw, Jw.T)))
#
#         iJ = np.linalg.pinv(J)
#         qd = np.dot(iJ, vwref.T)
#
#         print('Joint speed norm: ', np.linalg.norm(qd))
#         qd = np.dot(DELTA_TIME, qd)
#         q = q + qd
#         robot.set_arm_joint_target_positions(q)
#         robot.wait_till_joint_position_is_met(q)
#         q_rs.append(q)
#
#     plot_trajectories(q_rs)
#
#     robot.stop_arm()
#     scene.stop_simulation()
#
#
# if __name__ == "__main__":
#     main_loop()
