#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

The script is used to freely move the UR5 robot based on:
1) joint commands. Use the keys
    (1, q) to increment/decrement joint 1
    (2, w) to increment/decrement joint 2
    (3, e) to increment/decrement joint 3
    (4, r) to increment/decrement joint 4
    (5, t) to increment/decrement joint 5
    (6, y) to increment/decrement joint 6

2) v, w commands. Please specify a speedjoint commands. Use the keys

@Authors: Arturo Gil. arturo.gil@umh.es
          Universidad Miguel Hernandez de Elche

@Time: November 2021

"""
import time
import sim
import sys
import numpy as np

from artelib.tools import R2quaternion, compute_w_between_R
from artelib.ur5 import RobotUR5
from artelib.scene import Scene
import matplotlib.pyplot as plt
from pynput import keyboard

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0

actions = {'1': '0+',
           'q': '0-',
           '2': '1+',
           'w': '1-',
           '3': '2+',
           'e': '2-',
           '4': '3+',
           'r': '3-',
           '5': '4+',
           't': '4-',
           '6': '5+',
           'y': '5-',
           'o': 'open_gripper',
           'c': 'close_gripper'
           }


def init_simulation():
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
    else:
        print("Connection not successful")
        sys.exit("Connection failed,program ended!")

    armjoints = []
    gripper = []
    objects = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'UR5', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'UR5_joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'UR5_joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'UR5_joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'UR5_joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'UR5_joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'UR5_joint6', sim.simx_opmode_oneshot_wait)
    errorCode, gripper1 = sim.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', sim.simx_opmode_oneshot_wait)
    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)
    # errorCode, sphere = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_oneshot_wait)
    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    gripper.append(gripper1)

    # objects.append(sphere)
    robot = RobotUR5(clientID=clientID, wheeljoints=[],
                    armjoints=armjoints, base=robotbase,
                    end_effector=end_effector, gripper=gripper, target=target)
    scene = Scene(clientID=clientID, objects=objects)
    return robot, scene


delta_increment = 0.05  # rad
q = np.zeros(6)
press_exit = False
robot, scene = init_simulation()
# set initial position of robot
robot.set_arm_joint_target_positions(q, wait=True)


def plot_trajectories(q_rs):
    q_rs = np.array(q_rs)
    plt.figure()

    for i in range(0, 6):
        plt.plot(q_rs[:, i], label='q' + str(i + 1))
    plt.legend()
    plt.show(block=True)


def on_press(key):
    try:
        print('Key pressed: {0} '.format(key.char))
        caracter = key.char
        if caracter == 'o':
            robot.open_gripper(wait=True)
            return True
        elif caracter == 'c':
            robot.close_gripper(wait=True)
            return True
        # for the rest of actions, decode  decode action from actions dictionary
        acti = actions[key.char]
        index = int(acti[0])
        sign = acti[1]
        if sign == '+':
            q[index] += delta_increment
        else:
            q[index] -= delta_increment
        robot.set_arm_joint_target_positions(q, wait=False)
        robot.wait(1)
        [position, orientation] = robot.get_end_effector_position_orientation()
        print('Current q is: ', q)
        print('End effector position is: ', position)
        print('End effector orientation is: ', orientation)
        # time.sleep(0.1)

    except (AttributeError, KeyError):
        print('special key pressed: {0}'.format(
            key))
    return True


def on_release(key):
    print('Key released: {0}'.format(
        key))
    if key == keyboard.Key.esc:
        print('Exiting')
        exit()
        return False


if __name__ == "__main__":
    print('Use: 1, 2, 3, 4, 5, 6 to increment q_i')
    print('Use: q, w, e, r, t, y to decrement q_i')
    print('Use: o, c to open/close gripper')
    print('Use ESC to exit')
    # Collect events until released
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    robot.stop_arm()
    scene.stop_simulation()
