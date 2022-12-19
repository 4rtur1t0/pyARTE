#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script computes the inverse kinematic of the IRB140 robot and sends joint values to Coppelia to view
the results.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from sceneconfig.scene_configs_irb140 import init_simulation_ABBIRB140


def inverse_kinematics(robot, target_position, target_orientation):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion.

    caution: closest tooooo
    """
    q = robot.inversekinematics(target_position, target_orientation)
    return q


def view_solutions(robot, q):
    """
    q es un array de 6 filas x n columnas, donde n es el número de soluciones válidas de la cinemática inversa
    """
    n_valid_solutions = q.shape[1]
    print('SOLUCIONES DE LA CINEMÁTICA INVERSA: ')
    print(np.array_str(q, precision=2, suppress_small=True))
    print('HAY ', n_valid_solutions, ' SOLUCIONES VÁLIDAS')
    # observa las soluciones
    for i in range(n_valid_solutions):
        qi = q[:, i]
        T = robot.directkinematics(qi)
        print(100*'*')
        print('Solución: ', i)
        print('Valores articulares q: ', np.array_str(qi, precision=2, suppress_small=True))
        print('Posición y orientación alcanzadas (T): ')
        T.print_nice()


def reach_solutions(robot, q):
    n_valid_solutions = q.shape[1]
    n_in_range = 0
    for i in range(n_valid_solutions):
        print(i)
        qi = q[:, i]
        print(np.array_str(qi, precision=2, suppress_small=True))
        total, partial = robot.check_joints(qi)
        if total:
            n_in_range += 1
        if total:
            print('ALL JOINTS WITHIN RANGE!!')
            robot.set_joint_target_positions(qi, precision=True)
        else:
            print('ONE OR MORE JOINTS OUT OF RANGE!')
            print(partial)
    print('Found ', n_in_range, 'SOLUTIONS in RANGE, out of ', n_valid_solutions, ' VALID SOLUTIONS')


def irb140_ikine():
    robot, _ = init_simulation_ABBIRB140()

    # set initial position
    q0 = np.array([0, 0, 0, 0, 0, 0])
    robot.set_joint_target_positions(q0, precision=True)

    # 8 Válidas y 8 alcanzables
    # target_position = [0.4, 0.0, 0.8]
    # target_orientation = [0, np.pi/2, 0]
    target_position = [0.5, 0.0, 0.9]
    target_orientation = [0, np.pi/2, 0]

    q = inverse_kinematics(robot=robot, target_position=target_position,
                           target_orientation=Euler(target_orientation))

    # Son validas matemáticamente?
    view_solutions(robot, q)
    # son realizables físicamente?
    reach_solutions(robot, q)

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()


if __name__ == "__main__":
    irb140_ikine()

