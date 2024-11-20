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


def pick(robot, gripper):
    """
    COMPLETE EL CÓDIGO
    """
    return


def place(robot, gripper, frame, i):
    """
    COMPLETE EL CÓDIGO
    """
    return


def pick_and_place():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')

    # Connect to the gripper
    gripper = GripperRG2(simulation=simulation)
    gripper.start(name='/IRB140/RG2/RG2_openCloseJoint')
    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))

    # para usar la ventosa
    # gripper = SuctionPad(simulation=simulation)
    # gripper.start()
    # set the TCP of the suction pad
    # robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.11]), Euler([-np.pi/2, 0, 0])))

    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    # MOVEMOS AL ROBOT A UNA POSICIÓN INICIAL
    robot.moveAbsJ(q0, endpoint=True, precision=True)
    # VAMOS A REPETIR EL PROCESO PARA UN TOTAL DE PIEZAS (p. ej. 24)
    n_pieces = 24
    for i in range(n_pieces):
        # ESPERAMOS A QUE LA PIEZA CORTE EL SENSOR FOTOELÉCTRICO Y SE PARE LA CINTA TRANSPORTADORA
        while True:
            if conveyor_sensor.is_activated():
                break
            simulation.wait()
        # cogemos una pieza
        pick(robot, gripper)
        # la paletizamos
        place(robot, gripper, frame, i)
        # repetimos ad eternum
    # Stop arm and simulation
    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

