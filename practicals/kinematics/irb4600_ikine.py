#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script computes the inverse kinematic of the IRB140 robot and sends joint values to Coppelia to view
the results.

In this script the instructions moveL, moveJ or moveAbsJ are not used, since the objective i to observe
the different solutions to the inversekinematics problem.

@Authors: Arturo Gil
@Time: April 2022
@Revision: July 2023, Arturo Gil
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from robots.abbirb4600 import RobotABBIRB4600
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def inverse_kinematics(robot, target_position, target_orientation, show_target=True):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion.

    caution: closest tooooo
    """
    if show_target:
        frame = ReferenceFrame(simulation=robot.simulation)
        frame.start()
        T = HomogeneousMatrix(target_position, target_orientation)
        frame.set_position_and_orientation(T)

    q = robot.inversekinematics(target_position, target_orientation, extended=False)
    return q


def view_all_solutions(robot, q):
    """
    q es un array de 6 filas x n columnas, donde n es el número de soluciones válidas de la cinemática inversa
    """
    print('TODAS LAS SOLUCIONES DE LA CINEMÁTICA INVERSA: ')
    print(np.array_str(q, precision=2, suppress_small=True))
    if q.size > 0:
        n_valid_solutions = q.shape[1]
    else:
        n_valid_solutions = 0
    print('HAY ', n_valid_solutions, ' SOLUCIONES VÁLIDAS MATEMÁTICAMENTE')
    # observa las soluciones
    for i in range(n_valid_solutions):
        qi = q[:, i]
        T = robot.directkinematics(qi)
        print(100*'*')
        print('Solución: ', i)
        print('Valores articulares q: ', np.array_str(qi, precision=2, suppress_small=True))
        print('Posición y orientación alcanzadas (T): ')
        T.print_nice()


def filter_within_range(robot, q):
    """
    q es un array de 6 filas x n columnas, donde n es el número de soluciones válidas de la cinemática inversa
    esta funcion elimina las soluciones que no están en el rango articular.
    """
    print('ELIMINANDO SOLUCIONES FUERA DE RANGO ARTICULAR: ')
    if q.size > 0:
        n_valid_solutions = q.shape[1]
    else:
        n_valid_solutions = 0
    n_in_range = 0
    q_in_range = []
    for i in range(n_valid_solutions):
        qi = q[:, i]
        total, partial = robot.check_joints(qi)
        if total:
            q_in_range.append(qi)
            n_in_range += 1

    print('SE HAN ENCONTRADO ', n_in_range, 'SOLUCIONES EN RANGO ARTICULAR DEL TOTAL DE ', n_valid_solutions,
          ' SOLUCIONES VALIDAS')
    q_in_range = np.array(q_in_range).T
    print(np.array_str(q_in_range, precision=2, suppress_small=True))
    return q_in_range


def move_robot(robot, q):
    if q.size > 0:
        n_solutions = q.shape[1]
    else:
        n_solutions = 0
    print('COMANDANDO AL ROBOT A LAS SOLUCIONES')
    for i in range(n_solutions):
        print(i)
        qi = q[:, i]
        robot.moveAbsJ(q_target=qi, precision=True)


def irb4600_ikine():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB4600(simulation=simulation)
    robot.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    # set initial position
    q0 = np.array([0, 0, 0, 0, 0, 0])
    robot.moveAbsJ(q_target=q0, precision=True)

    target_position = Vector([1.2, 0.8, 1.0])
    target_orientation = Euler([np.pi/8, np.pi/8, 0])

    qinv = inverse_kinematics(robot=robot, target_position=target_position,
                           target_orientation=target_orientation)

    for i in range(qinv.shape[1]):
        Ttarget = HomogeneousMatrix(target_position, target_orientation)
        T = robot.directkinematics(qinv[:, i])
        Tdiff = Ttarget-T
        Tdiff.print_nice()

    # Visualizamos todas las soluciones
    view_all_solutions(robot, qinv)

    # Se eliminan aquellas que no estan en rango articular
    qinv = filter_within_range(robot, qinv)

    # Se comanda al robot a estas soluciones
    move_robot(robot, qinv)

    # Stop arm and simulation
    simulation.stop()


if __name__ == "__main__":
    irb4600_ikine()

