#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.
The demo represents a KUKA LBR IIWA robot trying to avoid collisions with a sphere.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.plottools import plot_vars, plot_xy
from artelib.tools import buildT, null_space
from sceneconfig.scene_configs import init_simulation_KUKALBR

DELTA_TIME = 50.0/1000.0


def move_null_space(robot, q0, dir, nsteps):
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
    samples = range(0, len(q_path))
    for i in samples:
        robot.set_joint_target_positions(q_path[i], precision=False)
        d = robot.get_min_distance_to_objects()
        ds.append(d)
    return ds, q_path


def maximize_distance_to_obstacles(robot, q):
    ds, qs = move_null_space(robot, q, '-', 200)
    index = np.argmax(ds)
    return qs[index]

def increase_distance_to_obstacles(robot, q):
    robot.set_joint_target_positions(q, precision=True)
    d1 = robot.get_min_distance_to_objects()
    J, Jv, Jw = robot.get_jacobian(q)
    qd = null_space(J, 6)
    q = q + DELTA_TIME*qd
    robot.set_joint_target_positions(q, precision=True)
    d2 = robot.get_min_distance_to_objects()
    if d2 >= d1:
        return qd
    else:
        return -qd



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
        vrep = compute_repulsion(pe=pe, ps=ps)
        vwref = vwref + vrep
        vwref = robot.adjust_vwref(vwref=vwref, error_dist=error_dist, error_orient=error_orient, vmax=vmax)
        if error_dist < 0.01 and error_orient < 0.02:
            print('Converged!!')
            break
        J, Jv, Jw = robot.get_jacobian(q)
        # compute joint speed to achieve the reference
        qda = robot.moore_penrose_damped(J, vwref)
        qdb = increase_distance_to_obstacles(robot, q)
        # a 30% of the speed is used to run away from obstacles
        qdb = 0.8*np.linalg.norm(qda)*qdb
        qd = qda + qdb
        [qd, _, _] = robot.check_speed(qd)
        q = q + np.dot(DELTA_TIME, qd)
        [q, _] = robot.apply_joint_limits(q)
        q_path.append(q)
        qd_path.append(qd)
    return q_path, qd_path


def follow_line_obstacle(robot, sphere):
    target_positions = [[0.5, 0.4, 0.7],  # initial in front of conveyor
                        [0.5, -0.4, 0.7]]  # drop the piece on the table
    target_orientations = [[0, np.pi/8, 0],
                           [0, np.pi/8, 0]]

    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])
    # EJERCICIO:
    # CAMBIE LA POSICIÓN DE LAS ESFERAS
    sphere.set_object_position([0.2, -0.25, 0.75])
    #sphere.set_object_position([0.5, 0.0, 0.5])
    #sphere.set_object_position([0.5, 0.0, 0.4])
    #sphere.set_object_position([0.35, 0.0, 0.4])

    # EJERCICIO: MUEVA AL ROBOT EN EL ESPACIO NULO Y
    # HALLE q0 que lo aleje lo más posible de los obstáculos
    #q0 = maximize_distance_to_obstacles(robot, q0)

    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    [q1_path, _] = inversekinematics4(robot=robot, sphere=sphere, target_position=target_positions[0],
                                      target_orientation=target_orientations[0], q0=q0)
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    [q2_path, _] = inversekinematics4(robot=robot, sphere=sphere, target_position=target_positions[1],
                                      target_orientation=target_orientations[1], q0=q1_path[-1])
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_trajectory(q1_path, precision='last')
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    robot.set_joint_target_trajectory(q2_path, precision='last')
    robot.wait(15)


def application():
    robot, sphere = init_simulation_KUKALBR()
    follow_line_obstacle(robot, sphere)
    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    application()
