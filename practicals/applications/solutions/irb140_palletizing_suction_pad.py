#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import compute_3D_coordinates
from artelib.vector import Vector
from artelib.rotationmatrix import RotationMatrix
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2, SuctionPad
from robots.objects import ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation


def pick(robot, gripper, frame):
    q0 = np.array([np.pi/4, np.pi/8, np.pi/8, 0, -np.pi/8, -np.pi/8])
    tp1 = Vector([0.6, 0.267, 0.3])  # approximation
    tp2 = Vector([0.6, 0.267, 0.235]) # pick
    to = Euler([0, np.pi, -np.pi/2])

    frame.show_target_points([tp1], [to])
    frame.show_target_points([tp2], [to])

    robot.moveAbsJ(q0, precision=True)
    gripper.open(precision=True)
    robot.moveJ(target_position=tp1, target_orientation=to, precision=False)
    robot.moveL(target_position=tp2, target_orientation=to, precision=True, vmax=0.1)
    gripper.close(precision=True)
    robot.moveL(target_position=tp1, target_orientation=to, precision=False)


def place(robot, gripper, frame, i):
    # define que piece length and a small gap
    piece_length = 0.08
    piece_gap = 0.002
    q0 = np.array([-np.pi/4, np.pi/8, np.pi/8, 0, -np.pi/2, -np.pi/2])
    # POSITION AND ORIENTATION OF THE PALLET
    T0m = HomogeneousMatrix(Vector([-0.15, -0.7, 0.15]), Euler([0, 0, 0]))
    # POSICION DE LA PIEZA i EN EL SISTEMA MÓVIL m (RELATIVA)
    pi = compute_3D_coordinates(index=i, n_x=3, n_y=3, n_z=2, piece_length=piece_length, piece_gap=piece_gap)
    # POSICION p0 INICIAL SOBRE EL PALLET
    p0 = pi + np.array([0, 0, 2.5 * piece_length])
    Tmp0 = HomogeneousMatrix(p0, Euler([0, np.pi, 0]))
    # POSICIÓN p1 EXACTA DE LA PIEZA (considerando la mitad de su lado)
    p1 = pi + np.array([0, 0, 0.5 * piece_length])
    Tmp1 = HomogeneousMatrix(p1, Euler([0, np.pi, 0]))

    # TARGET POINT 0 y 1
    T0 = T0m*Tmp0
    T1 = T0m*Tmp1

    frame.show_target_points([T0.pos()], [T0.R()])
    frame.show_target_points([T1.pos()], [T1.R()])

    robot.moveAbsJ(q0, precision=True)
    robot.moveJ(target_position=T0.pos(), target_orientation=T0.R(), precision='last')
    robot.moveL(target_position=T1.pos(), target_orientation=T1.R(), precision=True, vmax=0.2)
    gripper.open(precision=True)
    robot.moveL(target_position=T0.pos(), target_orientation=T0.R(), precision='low')


def pick_and_place():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    gripper = SuctionPad(simulation=simulation)
    gripper.start()
    # set the TCP of the suction pad
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.105]), Euler([-np.pi/2, 0, 0])))

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q0, precision=True)
    n_pieces = 24
    for i in range(n_pieces):
        while True:
            if conveyor_sensor.is_activated():
                break
            simulation.wait()
        pick(robot, gripper, frame)
        place(robot, gripper, frame, i)
    # Stop arm and simulation
    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

