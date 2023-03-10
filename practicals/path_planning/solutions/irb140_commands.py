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
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import path_planning_line
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def show_target_points(clientID, target_positions, target_orientations, wait_time=10):
        frame = ReferenceFrame(clientID=clientID)
        frame.start()
        for i in range(len(target_positions)):
            T = HomogeneousMatrix(target_positions[i], target_orientations[i])
            frame.set_position_and_orientation(T)
            frame.wait(wait_time)


# def get_closest_to(qa, qb):
#     """
#     From a column wise list of solutions in qa, find the closest to qb.
#     """
#     n_solutions = qa.shape[1]
#     distances = []
#     for i in range(n_solutions):
#         d = np.linalg.norm(qa[:, i]-qb)
#         distances.append(d)
#     distances = np.array(distances)
#     distances = np.nan_to_num(distances, nan=np.inf)
#     idx = np.argmin(distances)
#     dd = distances[idx]
#     if dd > 0.5:
#         print('NOT SMOOTHHHH')
#     return qa[:, idx]


# def filter_path(robot, q0, qs):
#     """
#     EJERCICIO:
#     ELIMINAR LAS SOLUCIONES FUERA DEL RANGO DE LAS ARTICULACIONES
#     COMENZANDO POR q0, CONSTRUIR UN CAMINO EN q SUAVE
#     """
#     q_traj = []
#     # eliminar articulaciones fuera de rango
#     for i in range(len(qs)):
#         if len(qs[i]) == 0:
#             print('ERROR: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
#             continue
#         qs[i] = robot.filter_joint_limits(qs[i])
#     # Encuentre las soluciones más cercanas
#     for i in range(len(qs)):
#         # print('Movement i: ', i)
#         if len(qs[i]) == 0:
#             print('ERROR: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
#             continue
#         qi = get_closest_to(qs[i], q0)
#         q0 = qi
#         q_traj.append(qi)
#     q_traj = np.array(q_traj).T
#     return q_traj


# def compute_joint_trajectory(robot, q0, tps, tos):
#     # encontrar todas las soluciones para cada uno de los target points
#     qs = []
#     for i in range(len(tps)):
#         qi = robot.inversekinematics(tps[i], tos[i], extended=True)
#         qs.append(qi)
#
#     qs = filter_path(robot, qs, q0)
#     return qs


def compute_workspace_trajectory(robot, q0, target_positions, target_orientations):
    Ti = robot.directkinematics(q0)
    tps = []
    tos = []
    for i in range(len(target_positions)):
        tpi, toi = path_planning_line(Ti.pos(), Ti.R(), target_positions[i], target_orientations[i])
        tps.extend(tpi)
        tos.extend(toi)
        Ti = HomogeneousMatrix(target_positions[i], target_orientations[i])
    return tps, tos


def irb140_ikine_line():
    # Start simulation
    simulation = Simulation()
    clientID = simulation.start()
    # Connect to the robot
    robot = RobotABBIRB140(clientID=clientID)
    robot.start()
    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))


    # TRAJ 1: Square at constant orientation OK
    # OJO: se debe especificar la posición inicial q0 adecuada
    # se debe valorar el resultado suave de la trayectoria.
    # set initial position
    # q0 = np.array([0, 0, 0, 0, -np.pi / 2, 0])
    # robot.set_joint_target_positions(q0, precision=True)
    #
    # target_positions = [Vector([0.6, -0.5, 0.8]),
    #                     Vector([0.6, -0.5, 0.3]),
    #                     Vector([0.6, 0.5, 0.3]),
    #                     Vector([0.6, 0.5, 0.8]),
    #                     Vector([0.6, 0, 0.8])]
    # target_orientations = [Euler([0, np.pi / 2, 0]),
    #                        Euler([0, np.pi / 2, 0]),
    #                        Euler([0, np.pi / 2, 0]),
    #                        Euler([0, np.pi / 2, 0]),
    #                        Euler([0, np.pi / 2, 0])]

    # TRAJ 2 # # OK: Square surrounding the robot
    # # Square 2, try to get back to the origin
    q0 = np.array([-np.pi/2, 0, 0, 0, -np.pi / 2, 0])

    # Ti = robot.directkinematics(q0)
    target_positions = [Vector([-0.5, -0.5, 0.3]),
                        Vector([0.5, -0.5, 0.3]),
                        Vector([0.5, 0.5, 0.3]),
                        Vector([-0.5, 0.5, 0.3])]
    target_orientations = [Euler([0, np.pi, 0]),
                           Euler([0, np.pi, 0]),
                           Euler([0, np.pi, 0]),
                           Euler([0, np.pi, 0])]

    # # TRAJ 3:  OK
    # q0 = np.array([np.pi/2, 0, 0, 0, -np.pi / 2, 0])
    # robot.set_joint_target_positions(q0, precision=True)
    # Ti = robot.directkinematics(q0)
    # target_positions = [Vector([0.0, 0.6, 0.4]),
    #                     Vector([0.0, 0.6, 0.4]),
    #                     Vector([0.0, 0.6, 0.4]),
    #                     Vector([0.0, 0.6, 0.4]),
    #                     Vector([0.0, 0.6, 0.4])]
    # target_orientations = [Euler([0, np.pi/2, 0]),
    #                        Euler([np.pi/2, np.pi/2, 0]),
    #                        Euler([0, np.pi/2, 0]),
    #                        Euler([-np.pi/2, 0, 0]),
    #                        Euler([0, np.pi/2, 0])]

    # tps, tos = compute_workspace_trajectory(robot=robot, q0=q0, target_positions=target_positions,
    #                                         target_orientations=target_orientations)

    # mostrar en Coppelia los target points calculados
    # show_target_points(clientID, tps, tos, wait_time=1)

    # q0 --> implica T
    # se recorren todos los tps
    robot.moveAbsJ(q0, precision=True)
    for i in range(len(target_positions)):
        robot.moveL(target_position=target_positions[i], target_orientation=target_orientations[i], precision=False)

    robot.moveAbsJ(q0, precision=True)
    for i in range(len(target_positions)):
        robot.moveJ(target_position=target_positions[i], target_orientation=target_orientations[i], precision=True)




    # qs = compute_joint_trajectory(robot=robot, q0=q0, tps=tps, tos=tos)



    # Stop arm and simulation
    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    irb140_ikine_line()

