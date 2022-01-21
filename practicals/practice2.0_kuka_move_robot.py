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
import numpy as np
from artelib.tools import T2quaternion
import matplotlib.pyplot as plt
from pynput import keyboard
# standard delta time for Coppelia, please modify if necessary
from sceneconfig.scene_configs import init_simulation_KUKALBR

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
           '7': '6+',
           'u': '6-',
           'o': 'open_gripper',
           'c': 'close_gripper'
           }

delta_increment = 0.05  # rad
q = np.zeros(7)
press_exit = False
robot, scene = init_simulation_KUKALBR()
# set initial position of robot
robot.set_joint_target_positions(q, precision=True)


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
        if caracter == 'o':
            robot.open_gripper(precision=True)
            return True
        elif caracter == 'c':
            robot.close_gripper(precision=True)
            return True
        elif caracter == 'z':
            print('ARM RESET')
            q = np.zeros(7)
            robot.set_joint_target_positions(q, precision=True)
            return True
        # for the rest of actions,  decode action from actions dictionary
        acti = actions[key.char]
        index = int(acti[0])
        sign = acti[1]
        if sign == '+':
            q[index] += delta_increment
        else:
            q[index] -= delta_increment
        robot.set_joint_target_positions(q, precision=False)
        robot.wait(1)
        [position, orientation] = robot.get_end_effector_position_orientation()
        T = robot.direct_kinematics(q)
        Q = T2quaternion(T)
        print('Current q is: ', q)
        print('End effector position is (p): ', position)
        print('End effector orientation is (alpha, betta, gamma): ', orientation)
        print('End effector T is: ', T)
        print('End effector Q is: ', Q)

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
