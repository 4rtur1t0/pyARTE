#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

You may use something like this to generate the indices and then compute the positions of the cubes
on the pallet.

def generate_indices(nx, ny, nz):
    indices = []
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                indices.append([i, j, k])
    return indices

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import compute_3D_coordinates
from artelib.vector import Vector
from artelib.rotationmatrix import RotationMatrix
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2, SuctionPad
from robots.objects import ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation


def pick(robot, gripper):
    q0 = np.array([0, 0, 0, 0, np.pi/2, 0])
    tp1 = Vector([0.6, 0.04, 0.3])      # approximation
    tp2 = Vector([0.6, 0.04, 0.21])     # pick
    to1 = Euler([0, np.pi, 0])
    to2 = Euler([0, np.pi, 0])
    robot.moveAbsJ(q0, precision=True, speed_factor=3.0)
    gripper.open()
    robot.moveJ(target_position=tp1, target_orientation=to1, precision=True, speed_factor=2.0)
    robot.moveL(target_position=tp2, target_orientation=to2, precision=True, speed_factor=2.0)
    gripper.close()
    robot.moveL(target_position=tp1, target_orientation=to1, precision=False)


def place(robot, gripper, i):
    # define que piece length and a small gap
    piece_length = 0.08
    piece_gap = 0.02
    q0 = np.array([0.0, 0.0, 0.0, 0.0, np.pi/2, 0.0])
    # POSITION AND ORIENTATION OF THE PALLETs
    pallet_position = Vector([-0.15, -0.65, 0.12])
    pallet_orientation = Euler([0, 0, 0])
    T0m = HomogeneousMatrix(pallet_position, pallet_orientation)
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
    robot.moveL(target_position=T1.pos(), target_orientation=T1.R(), precision=True)
    gripper.open()
    robot.moveL(target_position=T0.pos(), target_orientation=T0.R(), precision=True)


def pick_and_place():
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()
    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')

    # Connect to the gripper
    gripper = GripperRG2(simulation=simulation)
    gripper.start()
    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))

    # para usar la ventosa
    # gripper = SuctionPad(simulation=simulation)
    # gripper.start()
    # set the TCP of the suction pad
    # robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.11]), Euler([-np.pi/2, 0, 0])))

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q0, precision=True)
    n_pieces = 24
    for i in range(n_pieces):
        while True:
            if conveyor_sensor.is_activated():
                break
            robot.wait()
        pick(robot, gripper)
        place(robot, gripper, i)
    # Stop arm and simulation
    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

