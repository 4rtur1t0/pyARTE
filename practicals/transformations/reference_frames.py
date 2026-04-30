#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2023
"""
import numpy as np
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.objects import ReferenceFrame
from robots.simulation import Simulation

if __name__ == "__main__":
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    # Change position and orientation of the frame
    # using Vector and RotationMatrix
    position = Vector([0.5, 0.0, 0.3])
    orientation = RotationMatrix(np.array([[0, 1, 0],
                                           [-1, 0, 0],
                                           [0, 0, 1]]))
    # Show the target point
    frame.show_target_point(target_position=position, target_orientation=orientation)
    simulation.wait_time(5)
    # using Vector and Euler
    position = Vector([.6, 0, .6])
    orientation = Euler([np.pi/4, np.pi/4, np.pi/4])
    # Show the target point
    frame.show_target_point(target_position=position, target_orientation=orientation)
    simulation.wait_time(5)
    # using a HomogeneousMatrix
    T = HomogeneousMatrix([[0, 0, -1, 0.6],
                           [0, 1, 0, -0.3],
                           [1, 0, 0, 0.8],
                           [0, 0, 0, 1]])
    frame.show_target_point(target_position=T.pos(), target_orientation=T.R())
    simulation.wait_time(5)

    simulation.stop()


