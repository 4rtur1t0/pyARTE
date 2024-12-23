#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820_2.ttt scene before running this script.

The demo represents a KUKA LBR IIWA robot trying to maximize the distance to obstacles in the null
 space.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.tools import null_space
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation
from robots.objects import Sphere


def move_null_space(robot, q0, dir, nsteps):
    robot.set_joint_target_positions(q0, precision=True)
    # ok perform n movements in null space
    n_movements_in_null_space = nsteps
    q = q0
    q_path = []
    ds = []
    for i in range(0, n_movements_in_null_space):
        print('Movement number: ', i)
        J, Jv, Jw = robot.manipulator_jacobian(q)
        qd = null_space(J, 6)
        # EJERCICIO: COMPLETE LA FUNCIÓN


    return ds, q_path


def maximize_distance_to_obstacles(robot, q):
    ds, qs = move_null_space(robot, q, '-', 200)
    index = np.argmax(ds)
    return qs[index]


def follow_line_obstacle(robot, sphere):
    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])
    sphere.set_position([0.1, -0.4, 0.75])

    # EJERCICIO: MUEVA AL ROBOT EN EL ESPACIO NULO Y
    # HALLE q0 que lo aleje lo más posible de los obstáculos
    q0 = maximize_distance_to_obstacles(robot, q0)
    robot.set_joint_target_positions(q0, precision=True)
    robot.wait(15)


def application():
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotKUKALBR(clientID=clientID)
    robot.start()
    gripper = GripperRG2(clientID=clientID)
    gripper.start()
    sphere = Sphere(clientID=clientID)
    sphere.start()

    follow_line_obstacle(robot, sphere)
    # Stop arm and simulation
    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    application()
