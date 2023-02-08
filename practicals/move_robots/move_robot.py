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
"""
import numpy as np
import matplotlib.pyplot as plt
from pynput import keyboard
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation
from robots.ur5 import RobotUR5
from robots.abbirb140 import RobotABBIRB140
# from sceneconfig.scene_configs_irb140 import init_simulation_ABBIRB140


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


def on_press(key):
    global q
    try:
        print('Key pressed: {0} '.format(key.char))
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
        [q, _] = robot.apply_joint_limits(q)
        robot.set_joint_target_positions(q, precision=False)
        # robot.wait()
        # [position, orientation] = robot.get_end_effector_position_orientation()
        T = robot.directkinematics(q)
        Q = T.Q()
        print('Current q is: ', q)
        # print('End effector position is (p): ', position)
        # print('End effector orientation is (alpha, betta, gamma): ', orientation)
        print('End effector T is: ', T)
        print('End effector Q is: ', Q)

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
    clientID = simulation.start()

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
        robot = RobotABBIRB140(clientID=clientID)
    elif value == str(2):
        robot = RobotUR5(clientID=clientID)
    else:
        robot = RobotKUKALBR(clientID=clientID)

    robot.start()
    gripper = GripperRG2(clientID=clientID)
    gripper.start()
    robot.set_joint_target_positions(q, precision=True)

    # Collect events until released
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    simulation.stop()
