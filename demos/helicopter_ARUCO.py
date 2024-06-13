#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/helicopter_ARUCO.ttt scene before running this script.

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
from robots.objects import ReferenceFrame, CoppeliaObject
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
from robots.camera import Camera


def look_for_arucos(camera, show=True):
    """
    In case we want a list to all ARUCOS along with its transformation
    """
    id, Tca = camera.detect_closer_aruco(show=show)
    return id, Tca


def detect_arucos():
    simulation = Simulation()
    simulation.start()
    # frame = ReferenceFrame(simulation=simulation)
    # frame.start()
    camera = Camera(simulation=simulation)
    camera.start(name='/Helicopter/camera')
    helicopter = CoppeliaObject(simulation=simulation)
    helicopter.start(name='/Helicopter')

    M = 50
    posx = np.linspace(1, 0, M)
    posy = np.linspace(3, 0, M)
    posz = np.linspace(8, 0.1, M)

    # orientations

    for i in range(len(posz)):
        r = np.random.randn()/10
        helicopter.set_position_and_orientation(Vector([posx[i], posy[i], posz[i]]),
                                                Euler([np.pi/2+r, r, r]))
        simulation.wait()
        id, Tca = look_for_arucos(camera, show=True)
        # if id is None:
        #     print('NO ARUCO FOUND')
        #     break

    simulation.stop()


if __name__ == "__main__":
    detect_arucos()


