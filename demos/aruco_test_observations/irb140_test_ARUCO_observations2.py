#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_project_ARUCO.ttt scene before running this script.

The code beneath must be completed by the student in order to produce different palletizations of ARUCO pieces
according to their ids.

@Authors: Arturo Gil
@Time: February 2024
"""
import numpy as np
from artelib.euler import Euler
from artelib.path_planning import compute_3D_coordinates
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import get_object_transform, ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
from robots.camera import Camera
import matplotlib.pyplot as plt

def observe_arucos():
    simulation = Simulation()
    simulation.start()

    resolution = 1200
    fov = 45
    aruco_size = 0.1

    camera = Camera(simulation=simulation, resolution=resolution, fov_degrees=fov)
    camera.start(name='/camera')

    simulation.wait()
    id, Tca = camera.detect_closer_aruco(show=True, aruco_size=aruco_size)


    print(id)
    print(Tca)

    z_aruco = 0.05
    z_camara = 0.5
    error = 1000*np.abs(Tca.pos()-np.array([0, 0, z_camara-z_aruco]))
    print('error (mm): ', error)

    simulation.stop()


if __name__ == "__main__":
    observe_arucos()

