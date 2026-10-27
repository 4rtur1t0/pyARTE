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
from artelib.vector import Vector
from artelib.rotationmatrix import RotationMatrix
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2, SuctionPad
from robots.objects import ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation


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
    # gripper = GripperRG2(simulation=simulation)
    # gripper.start()
    # set the TCP of the RG2 gripper
    # robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))

    # para usar la ventosa
    gripper = SuctionPad(simulation=simulation)
    gripper.start()
    # set the TCP of the suction pad
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.105]), Euler([-np.pi/2, 0, 0])))

    # MOVEMOS AL ROBOT A UNA POSICIÓN INICIAL
    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q0, precision=True)
    # VAMOS A REPETIR EL PROCESO PARA UN TOTAL DE PIEZAS (p. ej. 24)
    n_pieces = 24
    for i in range(n_pieces):
        # ESPERAMOS A QUE LA PIEZA CORTE EL SENSOR FOTOELÉCTRICO Y SE PARE LA CINTA TRANSPORTADORA
        while True:
            if conveyor_sensor.is_activated():
                break
            robot.wait()
        ##########################################################################################################
        # EJERCICIO: se debe completar el código para completar la aplicación de paletizado.
        ##########################################################################################################

        ##########################################################################################################
        # A) función pick
        # COGEMOS UNA PIEZA: . --> necesitamos conocer la posición y orientación de la pieza en el
        # sistema de coordenadas de la base del robot.
        ##########################################################################################################
        ##########################################################################################################
        # B) función place
        # AHORA DESEAMOS DEJAR LA PIEZA sobre el PALLET. Primero calculamos la posición y orientación
        # de la pieza en el sistema de coordenadas del pallet. Después la transformamos al sistema de coordenadas
        # de la base del robot.
        ##########################################################################################################

        ##########################################################################################################
        # C) REPETIMOS SINE DIE o hasta que se acaben las piezas.
        ##########################################################################################################


    # Stop arm and simulation
    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

