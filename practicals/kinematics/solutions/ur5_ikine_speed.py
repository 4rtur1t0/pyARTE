#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: December 2023
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
    robot.moveAbsJ(q)
    T = robot.directkinematics(q)
    T.print_nice()
    print('Current T: ')
    # relative point
    v = np.array([0.0, 0.005, -0.3])
    pb = T.pos() + 2*v
    vw = np.array([v, [0, 0, 0]]).flatten()

    while True:
        q = robot.get_joint_positions()
        J, _, _ = robot.manipulator_jacobian(q)
        T = robot.directkinematics(q)
        e = np.linalg.norm(pb - T.pos())
        print(e)
        if e < 0.02:
            robot.set_joint_target_velocities([0, 0, 0, 0, 0, 0])
            break
        qd = np.dot(np.linalg.inv(J), vw.T)
        robot.set_joint_target_velocities(qd)
        robot.wait()

    print('POSITION REACHED', T.pos())
    print('ERROR: ', np.linalg.norm(T.pos()-pb))
    simulation.stop()


if __name__ == "__main__":
    compute_inverse_kinematics()

