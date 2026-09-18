#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_sorting.ttt scene before running this script.

The code beneath must be completed by the student in order to produce a piece sorting application. Place the different
pieces on the 4 conveyor belts.

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
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
from robots.camera import Camera


def sorting_operation():
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
    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')

    # Se mueve el robot a una posición articular q inicial
    q = np.array([0, 0, 0, 0, 0, 0])
    robot.moveAbsJ(q_target=q, precision=False, speed_factor=3.0)
    aruco_size = 0.05
    id, Tca = camera.detect_closer_aruco(show=True, aruco_size=aruco_size)
    if id is None:
        print('could not find piece')
    else:
        print('Found piece with id: ', id)
    # desactiva la ventosa
    suction.suction_off()
    tp = Vector([0.6, 0.0, 0.29])
    to = Euler([0, np.pi/8, 0])
    robot.moveJ(target_position=tp.pos() + np.array([0, 0, 0.2]), target_orientation=to, speed_factor=3.0)
    robot.moveL(target_position=tp.pos(), target_orientation=to, speed_factor=2.0, precision=True)
    # activa la ventosa
    suction.suction_on()
    robot.moveL(target_position=tp.pos() + np.array([0, 0, 0.2]), target_orientation=to, speed_factor=2.0)
    suction.suction_off()
    robot.wait_time(0.2)
    simulation.stop()


if __name__ == "__main__":
    sorting_operation()


