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
    clientID = simulation.start()
    frame = ReferenceFrame(clientID=clientID)
    frame.start()

    # Change position and orientation of the frame
    # using Vector and RotationMatrix
    position = Vector([0.5, 0, 0.3])
    orientation = RotationMatrix(np.array([[0, 1, 0],
                                           [-1, 0, 0],
                                           [0, 0, 1]]))
    frame.set_position(position=position)
    simulation.wait(50)
    frame.set_orientation(orientation=orientation)
    simulation.wait(50)
    # same as before
    frame.set_position_and_orientation(position, orientation)
    simulation.wait(50)

    # using Vector and Euler
    position = Vector([.6, 0, .6])
    orientation = Euler([np.pi/4, np.pi/4, np.pi/4])
    frame.set_position(position=position)
    simulation.wait(50)
    frame.set_orientation(orientation=orientation)
    simulation.wait(50)
    # same as before
    frame.set_position_and_orientation(position, orientation)
    simulation.wait(50)
    # using a HomogeneousMatrix
    T = HomogeneousMatrix([[0, 0, -1, 0.6],
                           [0, 1, 0, -0.3],
                           [1, 0, 0, 0.8],
                           [0, 0, 0, 1]])
    frame.set_position(position=T.pos())
    simulation.wait(50)
    frame.set_orientation(orientation=T.R())
    simulation.wait(50)

    frame.set_position_and_orientation(T)

    simulation.stop()


