#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_batt.ttt scene before running this script.

The code beneath must be completed by the student in order to produce an assembly operation of the battery.
Each battery must include three lithium cells.

@Authors: Arturo Gil
@Time: March 2026
"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import ReferenceFrame
from robots.simulation import Simulation
from robots.camera import Camera


def assembly_operation():
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=45)
    camera.start(name='/IRB140/camera')
    suction = SuctionPad(simulation=simulation)
    suction.start()
    # TCP DE LA VENTOSA! OJO!: debe ser el adecuado para la escena
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))

    # Se mueve el robot a una posición articular q inicial
    q = np.array([0, 0, 0, 0, 0, 0])
    robot.moveAbsJ(q_target=q, precision=False, speed_factor=3.0)

    # desactivamos la ventosa (no aspira)
    suction.suction_off()
    # nos movemos cerca de la pieza amarilla. La cámara debe ser capaz de observar la marca ARUCO con
    # un buen tamaño.
    tp1 = Vector([0.5, 0.15, 0.22])
    to1 = Euler([0, np.pi / 2, 0])
    robot.moveJ(target_position=tp1, target_orientation=to1)
    robot.moveL(target_position=tp1 + Vector([0, 0, 0.2]), target_orientation=to1)

    # a continuación, se calcula la relativa Tca
    aruco_size = 0.02
    id, Tca = camera.detect_closer_aruco(show=True, aruco_size=aruco_size)
    # se activa la succión
    suction.suction_on()
    # se desactiva la ventosa
    suction.suction_off()
    simulation.stop()


if __name__ == "__main__":
    assembly_operation()


