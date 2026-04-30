#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.

    EXERCISE: MOVE THE ROBOT ARBITRARILY IN THE NULL SPACE, considering
    n=7DOF
    m=6DOF, the task has 3DOF in position + 3DOF in orientation

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation


def null_space(robot):
    """
    Obtain a unit vector in the direction of the J null space.
    Consider m as DOF of the application of interest.
    """

    return qd


def move_null_space(robot):
    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])
    robot.moveAbsJ(q_target=q0, precision=True)
    n_movements_in_null_space = 150
    for i in range(0, n_movements_in_null_space):
        q = robot.get_joint_positions()
        print('Movement number: ', i)
        J, Jv, Jw = robot.manipulator_jacobian(q)
        qd = null_space(J)
        ######################################################################################################
        # CUIDADO: el movimiento definido por qd puede no ser suave o incluso ser errático
        # EJERCICIO: MUEVA EL ROBOT EN EL ESPACIO NULO CONSIDERANDO UNA VELOCIDAD positiva/negativa EN qd[2]
        ######################################################################################################
        robot.set_joint_target_velocities(qd)
        robot.wait()


if __name__ == "__main__":
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotKUKALBR(simulation=simulation)
    robot.start()
    move_null_space(robot)
    simulation.stop()
    robot.plot_trajectories()
