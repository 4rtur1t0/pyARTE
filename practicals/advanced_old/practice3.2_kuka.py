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
from artelib.plottools import plot_vars
from artelib.tools import buildT
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation
from sceneconfig.scene_configs_kukalbr import init_simulation_KUKALBR

DELTA_TIME = 50/1000


def null_space(J):
    """
    Obtain a unit vector in the direction of the J null space.
    Consider m as DOF of the application of interest.
    """

    return qd


def move_null_space(robot):
    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])
    robot.set_joint_target_positions(q0, precision=True)
    # ok perform n movements in null space
    n_movements_in_null_space = 150
    q=q0
    q_path = []
    qd_path = []
    for i in range(0, n_movements_in_null_space):
        print('Movement number: ', i)
        J, Jv, Jw = robot.manipulator_jacobian(q)
        qd = null_space(J)
        ######################################################################################################
        # CUIDADO: el movimiento definido por qd puede no ser suave o incluso ser errático
        # EJERCICIO: MUEVA EL ROBOT EN EL ESPACIO NULO CONSIDERANDO UNA VELOCIDAD positiva/negativa EN qd[2]
        ######################################################################################################



        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
        q_path.append(q)
        qd_path.append(qd)
    robot.set_joint_target_trajectory(q_path, precision='last')
    plot_vars(qd_path, 'JOINT SPEEDS')


if __name__ == "__main__":
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotKUKALBR(clientID=clientID)
    robot.start()

    move_null_space(robot)
    # Stop arm and simulation

    simulation.stop()
    robot.plot_trajectories()
