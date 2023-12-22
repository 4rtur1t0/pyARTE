#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.tools import buildT, compute_kinematic_errors
from robots.grippers import GripperRG2
from robots.simulation import Simulation
from robots.ur5 import RobotUR5
from artelib.vector import Vector
from artelib.euler import Euler
from artelib.rotationmatrix import RotationMatrix


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


def pick_and_place():
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))

    target_positions = [[0.6, -0.2, 0.25], # initial in front of conveyor
                        [0.6, 0.1, 0.25], # pick the piece
                        [0.6, 0.1, 0.35], # bring the piece up
                        [0.4, -0.1, 0.35], # middle point
                        [0.2, -0.55, 0.4], # over the table
                        [0.2, -0.55, 0.3]] # drop the piece
    target_orientations = [[-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi / 2, 0, 0],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0]]
    q0 = np.array([-np.pi, -np.pi/8, np.pi/2, 0, 0, 0])

    # plan trajectories
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

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.moveAbsJ(q0, precision=True)
    robot.wait(15)
    gripper.open(precision=True)
    # set the target we are willing to reach on Coppelia
    # robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.moveAbsJ(q1, precision=True)
    robot.moveAbsJ(q2, precision=True)
    gripper.close(precision=True)
    robot.moveAbsJ(q3, precision=True)
    robot.moveAbsJ(q4, precision=True)
    robot.moveAbsJ(q5, precision=True)
    robot.moveAbsJ(q6, precision=True)
    gripper.open(precision=True)
    robot.moveAbsJ(q5)

    # Stop arm and simulation
    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    pick_and_place()

