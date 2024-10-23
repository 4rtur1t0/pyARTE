#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script is used to freely move the UR5 robot based on:
1) joint commands. Use the keys
    (1, q) to increment/decrement joint 1
    (2, w) to increment/decrement joint 2
    (3, e) to increment/decrement joint 3
    (4, r) to increment/decrement joint 4
    (5, t) to increment/decrement joint 5
    (6, y) to increment/decrement joint 6

2) Open/close the RG2 gripper with keys o and c.

@Authors: Arturo Gil. arturo.gil@umh.es
          Universidad Miguel Hernandez de Elche

@Time: July 2022
@Time: November 2024 --> changed to opencv keyboard capture
"""
import time

import cv2
import numpy as np
import matplotlib.pyplot as plt
from pynput import keyboard
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation
from robots.ur5 import RobotUR5
from robots.abbirb140 import RobotABBIRB140


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

delta_increment = 0.05  # rad
q = np.zeros(6)
press_exit = False

global robot
global gripper

robot = None
gripper = None


def plot_trajectories(q_rs):
    q_rs = np.array(q_rs)
    plt.figure()
    for i in range(0, 6):
        plt.plot(q_rs[:, i], label='q' + str(i + 1))
    plt.legend()
    plt.show(block=True)

def move_robot():
    global q
    [q, _] = robot.apply_joint_limits(q)
    robot.moveAbsJ(q_target=q, precision=False, qdfactor=2.0)
    T = robot.directkinematics(q)
    Q = T.Q()
    print('Current q is: \n', q)
    print('End effector T is: \n', T)
    print('End effector position is (p): \n', T.pos())
    print('End effector orientation is (alpha, beta, gamma): \n', T.euler()[0])
    print('End effector Q is (Quaternion): ', Q)



def on_press(key):
    global q
    try:
        print('Key pressed: {0} '.format(key))
        try:
            caracter = key.char
            if caracter == 'o':
                gripper.open(precision=True)
                return True
            elif caracter == 'c':
                gripper.close(precision=True)
                return True
            elif caracter == 'z':
                print('ARM RESET')
                q = np.zeros(6)
                # robot.moveAbsJ(q_target=q, precision=True)
                return True
            # for the rest of actions,  decode action from actions dictionary
            acti = actions[key.char]
            index = int(acti[0])
            sign = acti[1]
            if sign == '+':
                q[index] += delta_increment
            else:
                q[index] -= delta_increment
        except:
            return

        # [q, _] = robot.apply_joint_limits(q)
        # robot.moveAbsJ(q_target=q, precision=False)
        # T = robot.directkinematics(q)
        # Q = T.Q()
        # print('Current q is: \n', q)
        # print('End effector T is: \n', T)
        # print('End effector position is (p): \n', T.pos())
        # print('End effector orientation is (alpha, beta, gamma): \n', T.euler()[0])
        # print('End effector Q is (Quaternion): ', Q)

    except (AttributeError, KeyError):
        print('special key pressed: {0}'.format(key))
    return True


def on_release(key):
    print('Key released: {0}'.format(key))
    if key == keyboard.Key.esc:
        print('Exiting')
        exit()
        return False


if __name__ == "__main__":
    simulation = Simulation()
    simulation.start()
    print('Use: 1, 2, 3, 4, 5, 6, 7 to increment q_i')
    print('Use: q, w, e, r, t, y, u to decrement q_i')
    print('Use: o, c to open/close gripper')
    print('Use ESC to exit')
    print("Robots:")
    print("1 IRB140")
    print("2 UR5")
    print("3 KUKA LBR")
    value = input("Please select a robot:\n")
    if value == str(1):
        print("IRB140 SELECTED")
        robot = RobotABBIRB140(simulation=simulation)
        robot.start()
        gripper = GripperRG2(simulation=simulation)
        gripper.start(name='/IRB140/RG2/RG2_openCloseJoint')
    elif value == str(2):
        print("UR5 SELECTED")
        robot = RobotUR5(simulation=simulation)
        robot.start()
        gripper = GripperRG2(simulation=simulation)
        gripper.start(name='/UR5/RG2/RG2_openCloseJoint')
    else:
        print("KUKA LBR SELECTED")
        robot = RobotKUKALBR(simulation=simulation)
        robot.start()
        gripper = GripperRG2(simulation=simulation)
        gripper.start(name='/LBR_iiwa_14_R820/RG2/RG2_openCloseJoint')
        q = np.zeros(7)


    while True:
        # The event listener will be running in this block
        with keyboard.Events() as events:
            # Block at most one second
            print('Press Key:')
            # time.sleep(0.01)
            event = events.get(0.05)
            if event is None:
                print('You did not press a key within one second')
                # move_robot()
                robot.wait()
                continue
            if event.key is keyboard.Key.esc:
                break
            else:
                print('Received event {}'.format(event))
                print('event: ', event)
                # print('Received key: ', event.char)
                print('Received key: ', event.key)
                on_press(event.key)
                move_robot()

    simulation.stop()
