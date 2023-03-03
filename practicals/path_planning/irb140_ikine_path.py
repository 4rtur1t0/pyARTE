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


def get_closest_to(qa, qb):
    """
    From a column wise list of solutions in qa, find the closest to qb.
    """
    n_solutions = qa.shape[1]
    distances = []
    for i in range(n_solutions):
        d = np.linalg.norm(qa[:, i]-qb)
        distances.append(d)
    distances = np.array(distances)
    distances = np.nan_to_num(distances, nan=np.inf)
    idx = np.argmin(distances)
    return qa[:, idx]


def inverse_kinematics_line(robot, target_position, target_orientation, q0, show_targets=True):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion in the robot.inversekinematics function

    caution: returns the closest solution to the specified q0
    """
    Ti = robot.directkinematics(q0)
    target_positions, target_orientations = path_planning_line(Ti.pos(), Ti.R(), target_position, target_orientation)

    if show_targets:
        show_target_points(robot.clientID, target_positions, target_orientations, wait_time=1)

    q = robot.inversekinematics_line(target_position, target_orientation, vmax=0.5, q0=q0)

    # EJERCICIO: ELIMINAR ELEMENTOS FUERA DE RANGO
    for i in range(len(q)):
        if len(q[i]) == 0:
            print('ERROR: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
        q[i] = robot.filter_joint_limits(q[i])
    q_traj = []
    for i in range(len(q)):
        try:
            qi = get_closest_to(q[i], q0)
            q0 = qi
            q_traj.append(qi)
        except:
            pass
    q_traj = np.array(q_traj).T
    return q_traj


def irb140_ikine_line():
    # Start simulation
    simulation = Simulation()
    clientID = simulation.start()
    # Connect to the robot
    robot = RobotABBIRB140(clientID=clientID)
    robot.start()
    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))

    # set initial position
    q0 = np.array([0, 0, 0, 0, 0, 0])
    robot.set_joint_target_positions(q0, precision=True)

    # 8 Válidas y 8 alcanzables
    target_positions = [Vector([0.5, 0.0, 0.9]),
                        Vector([0.5, 0.0, 0.9]),
                        Vector([0.5, -0.3, 0.9]),
                        Vector([0.5, 0.3, 0.9]),
                        Vector([0.5, 0.3, 0.4]),
                        Vector([0.5, 0.0, 0.9]),
                        Vector([0.5, 0.0, 0.9]),
                        Vector([0.5, 0.0, 0.9])]
    target_orientations = [Euler([0, np.pi / 2, 0]),
                           Euler([0, 0, 0]),
                           Euler([0, 0, 0]),
                           Euler([0, 0, np.pi/2]),
                           Euler([0, np.pi, 0]),
                           Euler([0, np.pi/2, 0]),
                           Euler([0, np.pi/2, np.pi/2]),
                           Euler([np.pi/2, np.pi/2, np.pi/2])]
    for i in range(len(target_positions)):
        Ti = robot.directkinematics(q0)
        tps, tos = path_planning_line(Ti.pos(), Ti.R(), target_positions[i], target_orientation)

    show_target_points(clientID, target_positions, target_orientations, wait_time=10)

    for i in range(len(target_positions)):
        q = inverse_kinematics_line(robot=robot, target_position=target_positions[i],
                                    target_orientation=target_orientations[i], q0=q0)
        q0 = q[:, -1]

        # Se comanda al    robot a estas soluciones
        robot.set_joint_target_positions(q, precision='last')

    # Stop arm and simulation
    simulation.stop()

    robot.plot_trajectories()


if __name__ == "__main__":
    irb140_ikine_line()

