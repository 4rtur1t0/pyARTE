#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_ARUCO.ttt scene before running this script.

This script tries to capture a series of images using the camera.

After that, the calibration procedure should be performed using the script
perform_camera_calibration.py.

@Authors: Arturo Gil
@Time: April 2024
"""
import numpy as np
from artelib.euler import Euler
from artelib.path_planning import compute_3D_coordinates
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import get_object_transform, ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
from robots.camera import Camera


def capture_calibration_patterns():
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
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    target_positions = [[0.1, -0.45, 0.5], # ok
                        [0.1, -0.45, 0.55], # ok
                        [0.1, -0.45, 0.6], # ok
                        [0.1, -0.30, 0.6],
                        [0.05, -0.31, 0.7],
                        [0.0, -0.3, 0.65]]
    target_orientations = [[0, np.pi, 0], # ok
                           [0, np.pi/1.01, 0], # ok
                           [0, -np.pi/1.01, 0], # ok
                           [2.93, 0.07, np.pi],
                           [2.8, 0.14, np.pi],
                           [2.8, 0.14, np.pi]]

    for i in range(len(target_positions)):
        robot.moveJ(target_position=target_positions[i],
                    target_orientation=target_orientations[i],
                    endpoint=True, precision=False)
        camera.save_image('calib_pattern' + str(i) + '.png')
    simulation.stop()


if __name__ == "__main__":
    capture_calibration_patterns()

