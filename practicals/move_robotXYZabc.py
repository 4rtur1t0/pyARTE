#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script is used to freely move a robot (the IRB 140, UR5 or KUKA IIWA) based on:
1) XYZ alpha, beta gamma (in global coordinates). Use the keys
    (1, q) to increment/decrement X
    (2, w) to increment/decrement Y
    (3, e) to increment/decrement Z
    (4, r) to increment/decrement alpha
    (5, t) to increment/decrement beta
    (6, y) to increment/decrement gamma

2) Open/close the RG2 gripper with keys o and c.

@Authors: Arturo Gil. arturo.gil@umh.es
          Universidad Miguel Hernandez de Elche

@Time: July 2022
"""
import numpy as np
import matplotlib.pyplot as plt
from pynput import keyboard
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation
from robots.ur5 import RobotUR5
from robots.abbirb140.abbirb140 import RobotABBIRB140
from artelib.rotationmatrix import RotationMatrix, Rx, Ry, Rz

actions = {'1': 'X+',
           'q': 'X-',
           '2': 'Y+',
           'w': 'Y-',
           '3': 'Z+',
           'e': 'Z-',
           '4': 'A+',
           'r': 'A-',
           '5': 'B+',
           't': 'B-',
           '6': 'C+',
           'y': 'C-',
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


def str_nice(array, precision=5):
    temp_array = array
    th = 0.0001
    idx = np.abs(temp_array) < th
    temp_array[idx] = 0
    return np.array_str(temp_array, precision=precision, suppress_small=True)


def plot_trajectories(q_rs):
    q_rs = np.array(q_rs)
    plt.figure()
    for i in range(0, 6):
        plt.plot(q_rs[:, i], label='q' + str(i + 1))
    plt.legend()
    plt.show(block=True)


def on_press(key):
    global q
    try:
        print('Key pressed: {0} '.format(key.char))
        caracter = key.char
        q = robot.get_joint_positions()
        T = robot.directkinematics(q)
        # if caracter == 'o':
        #     gripper.open()
        #     robot.wait(5)
        #     return True
        # elif caracter == 'c':
        #     gripper.close()
        #     robot.wait(5)
        #     return True
        if caracter == 'z':
            print('ARM RESET')
            q = np.zeros(6)
            robot.moveAbsJ(q_target=q, precision=True, speed_factor=5.0)
            return True
        # for the rest of actions,  decode action from actions dictionary
        acti = actions[key.char]
        command = acti[0]
        sign = acti[1]
        if sign == '+':
            delta = delta_increment
        else:
            delta = -delta_increment
        pos = T.pos()
        R = T.R()
        if command == 'X':
            pos = T.pos() + np.array([delta, 0, 0])
        elif command == 'Y':
            pos = T.pos() + np.array([0, delta, 0])
        elif command == 'Z':
            pos = T.pos() + np.array([0, 0, delta])
        elif command == 'A':
            R = R*Rx(delta)
        elif command == 'B':
            R = R*Ry(delta)
        elif command == 'C':
            R = R * Rz(delta)
        robot.moveJ(target_position=pos, target_orientation=R, precision=False)
        T = robot.directkinematics(q)
        Q = T.Q()
        # np.set_printoptions(precision=2, floatmode="fixed")
        print('Current q is: \n', q)
        print('End effector T is: \n')
        T.print()
        print('End effector position is (p): \n', str_nice(T.pos()))
        print('End effector orientation is (alpha, beta, gamma): \n', str_nice(T.euler()[0].abg))
        print('End effector Q is (Quaternion): ', Q)

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
        try:
            gripper = GripperRG2(simulation=simulation)
            gripper.start()
        except:
            print('NO RG2 GRIPPER FOUND.')
            pass
    elif value == str(2):
        print("UR5 SELECTED")
        robot = RobotUR5(simulation=simulation)
        robot.start()
        try:
            gripper = GripperRG2(simulation=simulation, joint_name='/UR5/RG2/RG2_openCloseJoint')
            gripper.start()
        except:
            print('NO RG2 GRIPPER FOUND.')
            pass
    else:
        print("KUKA LBR SELECTED")
        robot = RobotKUKALBR(simulation=simulation)
        robot.start()
        # gripper = GripperRG2(simulation=simulation, joint_name='/LBR_iiwa_14_R820/RG2/RG2_openCloseJoint')
        # gripper.start()
        q = np.zeros(7)

    # Collect events until released
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    simulation.stop()
