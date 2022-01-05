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


def moore_penrose(J, vwref):
    """
    Compute qd given:
        J: Manipulator Jacobian.
        (v, w): the linear and angular speed on the robots end-effector (in global coordinates)
    """
    #####################################################
    # EJERCICIO:
    # 1 Calcular la pseudo-inversa Jp = J^T(J*J^T)^{-1}
    # 2 Calcule qd = Jp*vwref
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
        # A) CONOCIDO vwref=(v, w) que lleva, en cada instante, el extremo hacia la Ttarget (posicion/orientacion)
        #  deseadas.
        #   CALCULAR qd usando Moore-Penrose (rellene la funcion moore_penrose. El método
        #   robot.get_jacobian nos devuelve la manipulator Jacobian evaluada en q.
        # B) Integrar qd para obtener la solución q.
        # C) Decidir cuando se termina el algoritmo.
        ###########################################################################################
    return q


def move_to_target():
    robot, scene = init_simulation_UR5()
    ################################################################
    # EJERCICIO:
    # A) DEBERÁ COMPLETAR LA FUNCIÓN inverse_kinematics
    # B) DEBERÁ COMPLETAR LA FUNCI moore_penrose
    # C) DEBERA AÑADIR mAs target points para realizar la tarea.
    ###############################################################
    # initial joint position q0
    q0 = np.array([-np.pi, 0.1, np.pi/2, 0.1, 0.1, 0.1])
    # a list of target positions
    target_positions = [[0.6, -0.2, 0.25],
                        [0.6, 0.1, 0.25]]
    # a list of target orientations (Euler angles alpha, beta, gamma in rad)
    target_orientations = [[-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2]]

    # Compute inverse kinematics for each targe position/orientation
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=target_orientations[0], q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=target_orientations[1], q0=q1)

    # NOW! execute trajectories on the simulated robot
    # set initial position of robot
    robot.set_joint_target_positions(q0, wait=True)
    robot.open_gripper()
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_positions(q1, wait=True)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    robot.set_joint_target_positions(q2, wait=True)
    robot.close_gripper(wait=True)
    robot.open_gripper(wait=True)
    # plot joints for each movement
    robot.plot_trajectories()
    # stop the arm
    robot.stop_arm()
    # stop simulation
    scene.stop_simulation()


if __name__ == "__main__":
    move_to_target()

