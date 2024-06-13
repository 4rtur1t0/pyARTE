#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/irb140_project_ARUCO.ttt scene before running this script.

The code beneath must be completed by the student in order to produce different palletizations of ARUCO pieces
according to their ids.

@Authors: Arturo Gil
@Time: February 2024
"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
from robots.camera import Camera


def look_for_arucos(robot, camera, show=True):
    """
    In case we want a list to all ARUCOS along with its transformation
    """
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    robot.moveJ(target_position=Vector([0.5, 0.05, 0.35]),
                target_orientation=Euler([0, np.pi, 0]))
    id, Tca = camera.detect_closer_aruco(show=show)
    return id, Tca


# def compute_target_point(robot, frame, Tca):
#     return tp


# def pick_aruco(robot, gripper, frame, tp):
# [...]


# def place_aruco(robot, gripper):
# [...]


def pick_and_place_arucos():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')
    camera = Camera(simulation=simulation)
    camera.start(name='/IRB140/camera')
    gripper = SuctionPad(simulation=simulation)
    gripper.start()
    # TCP DE LA VENTOSA! OJO!: debe ser el adecuado para la escena
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))

    for i in range(6):
        while True:
            simulation.wait()
            if conveyor_sensor.is_activated():
                break
        id, Tca = look_for_arucos(robot, camera, show=True)
        if id is None:
            print('NO ARUCO FOUND')
            break
        # [... ]
        # Compute the target point considering all the transformations
        # tp = compute_target_point(robot, frame, Tca)
        # pick_aruco(robot, gripper, frame, tp)
        # place_aruco(robot, gripper)

    simulation.stop()


if __name__ == "__main__":
    pick_and_place_arucos()


