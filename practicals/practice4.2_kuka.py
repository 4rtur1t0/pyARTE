#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820_2.ttt scene before running this script.

The demo represents a KUKA LBR IIWA robot trying to avoid collisions with a sphere.
The min distance of the robot to all obstacles is plotted and showed on the robot.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.plottools import plot_vars, plot_xy
from artelib.tools import buildT, null_space
from sceneconfig.scene_configs import init_simulation_KUKALBR

DELTA_TIME = 50.0/1000.0


def move_null_space(robot, q0, dir, nsteps):
    # EJERCICIO: DEBE USAR LA FUNCIÓN d = robot.get_min_distance_to_objects() QUE DEVUELVE
    # la distancia mínima del brazo a todos los objetos del entorno.
    # debe devolver un array con todas las distancias mínimas para los distintos instantes de simulación.
    robot.set_joint_target_positions(q0, precision=True)
    # ok perform n movements in null space
    n_movements_in_null_space = nsteps
    q = q0
    q_path = []
    ds = []
    for i in range(0, n_movements_in_null_space):
        print('Movement number: ', i)
        J, Jv, Jw = robot.get_jacobian(q)
        qd = null_space(J, 6)
        if dir == '+' and qd[2] < 0:
            qd = -qd
        elif dir == '-' and qd[2] > 0:
            qd = -qd
        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
        [q, out_of_range] = robot.apply_joint_limits(q)
        if out_of_range:
            break
        q_path.append(q)
        robot.set_joint_target_positions(q, precision=False)
    ##############################################################################
    # EJERCICIO: devuelva un array de distancias ds y un array de posiciones
    # articulares q_path
    ##############################################################################
    return ds, q_path


def find_min_distance(robot, q):
    ds, qs = move_null_space(robot, q, '-', 200)
    ##############################################################################
    # EJERCICIO: halle el índice index que maximiza las distancias en el array ds
    # La función devuelve la posición articular q que maximiza esta distancia
    #############################################################################
    return qs[index]


def potential(r):
    K = 0.5
    rs = 0.15 # radius of the sphere
    rmax = 0.3
    if r < rs:
        r = rs
    p = K * (1 / r - 1 / rmax)
    if p < 0:
        p = 0
    return p


def compute_repulsion(pe, ps):
    u = pe - ps
    r = np.linalg.norm(u)
    if r > 0.0:
        u = u / r
    p = potential(r)
    vrep = np.dot(p, u)
    vrep = np.hstack((vrep, np.array([0, 0, 0])))
    return vrep


def inversekinematics4(robot, sphere, target_position, target_orientation, q0, vmax=0.5):
    """
    fine: whether to reach the target point with precision or not.
    vmax: linear velocity of the planner.
    """
    Ttarget = buildT(target_position, target_orientation)
    q = q0
    max_iterations = 300
    q_path = []
    qd_path = []
    ps = sphere.get_position()
    Ti = robot.direct_kinematics(q)
    total_time = robot.compute_time(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax)
    total_time = 0.5*total_time
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        pe = Ti[0:3, 3]
        # compute ATTRACTION
        vwref, error_dist, error_orient = robot.compute_actions(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax,
                                                                total_time=total_time)
        # compute REPULSION
        vrep = compute_repulsion(pe=pe, ps=ps)
        vwref = vwref + vrep
        vwref = robot.adjust_vwref(vwref=vwref, error_dist=error_dist, error_orient=error_orient, vmax=vmax)
        if error_dist < robot.max_error_dist_inversekinematics and \
                error_orient < robot.max_error_orient_inversekinematics:
            print('Converged!!')
            break
        J, Jv, Jw = robot.get_jacobian(q)
        # compute joint speed to achieve the reference
        qd = robot.moore_penrose_damped(J, vwref)
        [qd, _, _] = robot.check_speed(qd)
        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
        [q, _] = robot.apply_joint_limits(q)
        q_path.append(q)
        qd_path.append(qd)
    return q_path, qd_path


def follow_line_obstacle(robot, sphere):
    target_positions = [[0.5, 0.4, 0.5],  # initial in front of conveyor
                        [0.5, -0.4, 0.5]]  # drop the piece on the table
    target_orientations = [[0, np.pi/8, 0],
                           [0, np.pi/8, 0]]

    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])

    # cambie la posición de la esfera
    sphere.set_object_position([0.1, -0.3, 0.75])
    #sphere.set_object_position([0.5, 0.0, 0.5])
    #sphere.set_object_position([0.5, 0.0, 0.4])
    #sphere.set_object_position([0.35, 0.0, 0.4])

    q0 = find_min_distance(robot, q0)

    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    [q1_path, _] = inversekinematics4(robot=robot, sphere=sphere, target_position=target_positions[0],
                                      target_orientation=target_orientations[0], q0=q0)
    [q2_path, _] = inversekinematics4(robot=robot, sphere=sphere, target_position=target_positions[1],
                                      target_orientation=target_orientations[1], q0=q1_path[-1])

    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_trajectory(q1_path, precision='last')
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    robot.set_joint_target_trajectory(q2_path, precision='last')
    robot.wait(15)


if __name__ == "__main__":
    robot, sphere = init_simulation_KUKALBR()
    follow_line_obstacle(robot, sphere)
    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()
