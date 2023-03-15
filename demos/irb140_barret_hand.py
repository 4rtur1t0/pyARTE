#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_barret_hand.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import compute_3D_coordinates
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import GripperBarretHand
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation


def pick(robot, gripper):
    tp1 = Vector([0.4, 0.27, 0.25])  # approximation
    tp2 = Vector([0.78, 0.27, 0.25]) # pick
    to = Euler([0, np.pi/2, np.pi/2])

    q0 = np.array([0, 0, 0, 0, 0, 0])
    robot.moveAbsJ(q_target=q0)
    gripper.open(precision=True)
    robot.moveJ(target_position=tp1, target_orientation=to)
    robot.moveL(target_position=tp2, target_orientation=to, precision='last')
    gripper.close(precision=True)
    robot.moveL(target_position=tp1, target_orientation=to)


def place(robot, gripper, i):
    # define que piece length and a small gap
    piece_length = 0.08
    piece_gap = 0.02
    q0 = np.array([0, 0, 0, 0, 0, 0])
    # POSITION AND ORIENTATION OF THE PALLET
    T0m = HomogeneousMatrix(Vector([-0.15, -0.65, 0.1]), Euler([0, 0, 0]))
    # POSICION DE LA PIEZA i EN EL SISTEMA MÓVIL m (RELATIVA)
    pi = compute_3D_coordinates(index=i, n_x=3, n_y=4, n_z=2, piece_length=piece_length, piece_gap=piece_gap)
    # POSICION p0 INICIAL SOBRE EL PALLET
    p0 = pi + np.array([0, 0, 2.5 * piece_length])
    Tmp0 = HomogeneousMatrix(p0, Euler([0, np.pi, 0]))
    # POSICIÓN p1 EXACTA DE LA PIEZA (considerando la mitad de su lado)
    p1 = pi + np.array([0, 0, 0.5 * piece_length])
    Tmp1 = HomogeneousMatrix(p1, Euler([0, np.pi, 0]))

    # TARGET POINT 0 y 1
    T0 = T0m*Tmp0
    T1 = T0m*Tmp1

    robot.moveAbsJ(q0, precision=True)
    robot.moveJ(target_position=T0.pos(), target_orientation=T0.R(), precision=True)
    robot.moveL(target_position=T1.pos(), target_orientation=T1.R(), precision='last')
    gripper.open(precision=True)
    robot.moveL(target_position=T0.pos(), target_orientation=T0.R(), precision='last')


def pick_and_place():
    # Start simulation
    simulation = Simulation()
    clientID = simulation.start()
    # Connect to the robot
    robot = RobotABBIRB140(clientID=clientID)
    robot.start()
    # Connect to the proximity sensor
    conveyor_sensor = ProxSensor(clientID=clientID)
    conveyor_sensor.start()
    # Connect to the gripper
    gripper = GripperBarretHand(clientID=clientID)
    gripper.start()
    # set the TCP of the Barret gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.3]), RotationMatrix(np.eye(3))))

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q_target=q0)
    n_pieces = 48
    for i in range(n_pieces):
        while True:
            if conveyor_sensor.is_activated():
                break
            robot.wait()

        robot.moveAbsJ(q_target=q0)
        pick(robot, gripper)
        robot.moveAbsJ(q_target=q0)
        place(robot, gripper, i)

    # Stop arm and simulation
    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

