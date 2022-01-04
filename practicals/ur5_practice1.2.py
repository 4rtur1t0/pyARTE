#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.tools import buildT
from sceneconfig.scene_configs import init_simulation_UR5

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


def moore_penrose_damped(J, vwref):
    """
    Compute qd given J and v.
    If close to singularity, used damped version.
    """
    # NOTA: np.linalg.det es el determinante de J*J^T
    manip = np.linalg.det(np.dot(J, J.T))
    print('Manipulability is: ', manip)
    #####################################################
    # EJERCICIO:
    # CASO NORMAL: Calcular la pseudo-inversa J^T(J*J^T)^{-1}
    #########################################################
    if manip > .01 ** 2:
        #Implementar solución Moore Penrose normal
        return qd

    # Moore penrose damped pseudo inverse J^T(J*J^T+kI)^{-1}
    print('Close to singularity: implementing DAMPED Least squares solution')
    #####################################################
    # EJERCICIO:
    # CASO SINGULAR: Calcular la pseudo-inversa J^T(J*J^T+KI)^{-1}
    #########################################################

    return qd


def inverse_kinematics(robot,
                       target_position,
                       target_orientation, q0):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    """
    Ttarget = buildT(target_position, target_orientation)
    q = q0
    max_iterations = 1500
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        J, Jv, Jw = robot.get_jacobian(q)
        vwref, error_dist, error_orient = robot.compute_actions(Tcurrent=Ti, Ttarget=Ttarget, vmax=1)
        print('vwref: ', vwref)
        print('errordist, error orient: ', error_dist, error_orient)
        ###########################################################################################
        # EJERCICIO:
        # Usando este esquema, se debe:
        # A) CONOCIDO (v, w) que lleva el extremo hacia la Ttarget (posicion/orientacion) deseadas.
        #   CALCULAR qd usando Moore-Penrose (rellene la funcion moore_penrose_damped. El método
        #   robot.get_jacobian nos devuelve la manipulator Jacobian evaluada en q.
        # B) Decidir cuando se termina el algoritmo.
        # C) Integrar qd para obtener la solución q.
        ###########################################################################################
    return q


def pick_and_place():
    robot, scene = init_simulation_UR5()
    ################################################################
    # EJERCICIO: AÑADA mas target points para realizar la tarea.
    #
    #
    ###############################################################
    target_positions = [[0.6, -0.2, 0.25], # initial in front of conveyor
                        [0.6, 0.1, 0.25]] # drop the piece
    target_orientations = [[-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2]]

    q0 = np.array([-np.pi, 0, np.pi/2, 0, 0, 0])
    # set initial position of robot
    robot.set_arm_joint_target_positions(q0, wait=True)

    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    # plan trajectories
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=target_orientations[0], q0=q0)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=target_orientations[1], q0=q1)
    ###############################################################################
    # EJERCICIO: calcule las q para los diferentes target points.
    ###############################################################################


    ###########################################################################
    #   EJERCICIO, use la función robot.follow_q_trajectory para llevar al robot
    #   a los target points calculados por inverse_kinematics.
    #############################################################################
    # execute trajectories
    robot.open_gripper()
    robot.follow_q_trajectory([q1])
    robot.follow_q_trajectory([q2])
    robot.close_gripper(wait=True)
    # robot.follow_q_trajectory([q3])
    robot.open_gripper(wait=True)

    robot.plot_trajectories()
    robot.stop_arm()
    scene.stop_simulation()


if __name__ == "__main__":
    pick_and_place()

