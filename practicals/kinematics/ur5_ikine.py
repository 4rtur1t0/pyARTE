#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.tools import compute_kinematic_errors
from artelib.vector import Vector
from artelib.rotationmatrix import RotationMatrix
from robots.simulation import Simulation
from robots.ur5 import RobotUR5


def delta_q_transpose(J, e):

    return dq

def moore_penrose(J, e):
    """
    Compute qd given J and v.
    If close to singularity, used damped version.
    """
    manip = np.linalg.det(np.dot(J, J.T))
    print('Manip is: ', manip)

    return qd

def moore_penrose_damped(J, e):
    """
    Compute qd given J and v.
    If close to singularity, used damped version.
    """
    manip = np.linalg.det(np.dot(J, J.T))
    print('Manip is: ', manip)

    return qd


def inverse_kinematics(robot, target_position, target_orientation, q0):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion.
    """
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    q = q0
    max_iterations = 10000
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.directkinematics(q)
        J, Jv, Jw = robot.manipulator_jacobian(q)
        e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
        print('Error: ', e)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        # EJERCICIO: CALCULE LA ACTUALIZACIÓN DEL MÉTODO BASADO EN LA JACOBIANA
        # PRUEBE LOS TRES MÉTODOS:
        # Moore-Penrose básico.
        # Moore-Penrose damped.
        # Traspuesta

        # opcional, aplique una restricción del movimiento
        #[q, _] = robot.apply_joint_limits(q)
    return q


def compute_inverse_kinematics():
    """
    Check direct and inverse kinematics
    Using Gradient descent/Jacobian based kinematics
    """
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))
    robot.start()

    # Find an initial T
    q = np.array([-np.pi/2, 0.3, 0.3, 0.3, 0.3, 0.3])
    T = robot.directkinematics(q)
    print('Current T: ')
    T.print_nice()
    # try to find a solution
    q0 = np.array([-0.1, -0.1, -0.1, -0.1, -0.1, -0.1])
    # EJERCICIO: COMPLETESE ESTA FUNCIÓN
    qinv = inverse_kinematics(robot=robot,
                              target_position=T.pos(),
                              target_orientation=T.R(), q0=q0)

    print('CHECKING CONSISTENCY')
    print('FOUND solution qinv: ', qinv)
    T_reached = robot.directkinematics(qinv)
    print('T reached : ')
    T_reached.print_nice()
    Tdiff = T - T_reached
    print('Difference in T')
    Tdiff.print_nice()

    print('Difference in the solutions')
    print(q-qinv)

    robot.moveAbsJ(q)
    robot.moveAbsJ(qinv)

    simulation.stop()


if __name__ == "__main__":
    compute_inverse_kinematics()

