#!/usr/bin/env python
# encoding: utf-8
"""
Codigo para proyecto transversal
Robot UR5 jugando a los bolos
scenes/more/bowling.ttt
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.tools import compute_kinematic_errors
from sceneconfig.scene_configs_ur5 import init_simulation_UR5BarrettHand


def delta_q_t(J, e):
    alpha1 = np.dot(np.dot(np.dot(J, J.T), e), e)
    alpha2 = np.dot(np.dot(J, J.T), e)
    alpha2 = np.dot(alpha2, alpha2)
    alpha = alpha1/alpha2
    dq = alpha*np.dot(J.T, e)
    return dq

#Calcular la jacobiana moore penrose
def moore_penrose_damped(J, e):
    manip = np.linalg.det(np.dot(J, J.T))
    #Si la manipulabilidad es mayor que 0.01^2 se utiliza la moore penrose pseudo inversa
    if manip > .01 ** 2:
        iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        qd = np.dot(iJ, e.T)
        return qd
    #Si la manipulabilidad es menor que 0.01^2 se utiliza la Damped
    K = 0.01 * np.eye(np.min(J.shape))
    iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + K))
    qd = np.dot(iJ, e.T)
    return qd

#Calcular la cinematica inversa
def inverse_kinematics(robot, target_position, target_orientation, q0):
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    q = q0
    max_iterations = 10000
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.directkinematics(q)
        J, Jv, Jw = robot.get_jacobian(q)
        e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
        print('vwref: ', e)
        print('errordist, error orient: ', error_dist, error_orient)
        #Sie el error es menor que 0.01 finaliza
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        qd = delta_q_t(J, e)
        q = q + qd
        [q, _] = robot.apply_joint_limits(q)
    return q

#Funcion para calcular los movimientos del robot
def posicionamiento():
    robot = init_simulation_UR5BarrettHand()
    target_positions = [[0.31, 0.22, 0.55], #Posicion delante de la bola
                        [0.31, 0.22, 0.55], #Coger la bola
                        [0.31, 0.22, 0.65], #Levantarla y girar
                        [0.31, 0.22, 0.65], #Giro para el final
                        [0.31, 0.02, 0.65], #Punto intermedio 1
                        [0.1, -0.25, 0.65], #Punto intermedio 2
                        [0.1, -0.505, 0.65], #Punto intermedio 3
                        [0.06, -0.505, 0.5], #Punto Final
                        [0.06, -0.505, 0.5],
                        [-0.1, -0.505, 0.5],
                        [0.06, -0.505, 0.5]] #Cambio en la orientacion
    target_orientations = [[-np.pi/2, 0, -np.pi],
                           [-np.pi/2, 0, -np.pi],
                           [-np.pi/2, 0, -1.3],
                           [-np.pi/2, np.pi/2, -1.3],
                           [-np.pi/2, np.pi/2, -1.3],
                           [-np.pi/2, np.pi/2, -1.3],
                           [-np.pi/2, np.pi/2, -1.3],
                           [-np.pi/2, np.pi/2, -1.3],
                           [-np.pi/2, 1.51, -np.pi/2],
                           [-np.pi/2, 1.51, -np.pi/2],
                           [-np.pi/2, 1.51, -np.pi/2]]

    q0 = np.array([-np.pi, -np.pi/4, np.pi/2, 0, 0, 0])

    #Planificacion de trayectorias
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=Euler(target_orientations[0]), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=Euler(target_orientations[1]), q0=q1)
    q3 = inverse_kinematics(robot=robot, target_position=target_positions[2],
                            target_orientation=Euler(target_orientations[2]), q0=q2)
    q4 = inverse_kinematics(robot=robot, target_position=target_positions[3],
                            target_orientation=Euler(target_orientations[3]), q0=q3)
    q5 = inverse_kinematics(robot=robot, target_position=target_positions[4],
                            target_orientation=Euler(target_orientations[4]), q0=q4)
    q6 = inverse_kinematics(robot=robot, target_position=target_positions[5],
                            target_orientation=Euler(target_orientations[5]), q0=q5)
    q7 = inverse_kinematics(robot=robot, target_position=target_positions[6],
                            target_orientation=Euler(target_orientations[6]), q0=q6)
    q8 = inverse_kinematics(robot=robot, target_position=target_positions[7],
                            target_orientation=Euler(target_orientations[7]), q0=q7)
    q9 = inverse_kinematics(robot=robot, target_position=target_positions[8],
                            target_orientation=Euler(target_orientations[8]), q0=q8)
    q10 = inverse_kinematics(robot=robot, target_position=target_positions[9],
                            target_orientation=Euler(target_orientations[9]), q0=q9)
    q11 = inverse_kinematics(robot=robot, target_position=target_positions[10],
                            target_orientation=Euler(target_orientations[10]), q0=q10)


    #Ejecutar las trayectorias anteriores
    robot.set_joint_target_positions(q0, precision=True)
    robot.wait(15)
    robot.open_gripper(precision=True)
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    robot.close_gripper(precision=True)
    robot.set_joint_target_positions(q3, precision=True)
    robot.set_joint_target_positions(q4, precision=True)
    robot.set_joint_target_positions(q5, precision=True)
    robot.set_joint_target_positions(q6, precision=True)
    robot.set_joint_target_positions(q7, precision=True)
    robot.set_joint_target_positions(q8, precision=True)
    robot.set_joint_target_positions(q9, precision=True)
    robot.set_joint_target_positions(q10, precision=True)
    robot.wait(20)
    robot.set_joint_target_positions(q11, precision=True)
    robot.open_gripper(precision=True)
    #Tiempo de espera para visualizar el lanzamiento
    robot.wait(300)
    robot.stop_arm()
    robot.stop_simulation()
    #Graficar las trayectorias articulares
    robot.plot_trajectories()


if __name__ == "__main__":
    posicionamiento()


