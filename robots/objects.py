#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage different objects in Coppelia simulations

@Authors: Arturo Gil
@Time: April 2023

"""
import numpy as np
from artelib import rotationmatrix
from artelib import homogeneousmatrix
from artelib import vector
from artelib import euler
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.vector import Vector


class CoppeliaObject():
    def __init__(self, simulation):
        self.simulation = simulation
        self.handle = None

    def start(self, name='/CoppeliaObject'):
        # Get the handles of the relevant objects
        handle = self.simulation.sim.getObject(name)
        self.handle = handle

    def set_position(self, position):
        if isinstance(position, np.ndarray):
            position = np.array(position)
        elif isinstance(position, list):
            position = np.array(position)
        elif isinstance(position, vector.Vector):
            position = position.array
        elif isinstance(position, homogeneousmatrix.HomogeneousMatrix):
            position = position.pos
        self.simulation.sim.setObjectPosition(self.handle, -1, position.tolist())
        self.simulation.wait()

    def set_orientation(self, orientation):
        # if orientation is given by a RotationMatrix
        if isinstance(orientation, rotationmatrix.RotationMatrix):
            abg = orientation.euler()[0]
            abg = abg.abg
        elif isinstance(orientation, list):
            abg = np.array(orientation)
        elif isinstance(orientation, euler.Euler):
            abg = orientation.abg
        self.simulation.sim.setObjectOrientation(self.handle, -1, abg.tolist())
        self.simulation.wait()

    def set_position_and_orientation(self, *args):
        if len(args) == 1:
            if isinstance(args[0], homogeneousmatrix.HomogeneousMatrix):
                position = args[0].pos()
                orientation = args[0].R().euler()[0].abg
        elif len(args) == 2:
            position = args[0]
            orientation = args[1]
            if isinstance(position, list):
                position = np.array(position)
            elif isinstance(position, vector.Vector):
                position = np.array(position.array)
            if isinstance(orientation, euler.Euler):
                orientation = orientation.abg
            elif isinstance(orientation, rotationmatrix.RotationMatrix):
                orientation = orientation.euler()[0].abg
            else:
                raise Exception

        self.simulation.sim.setObjectPosition(self.handle, -1, position.tolist())
        self.simulation.sim.setObjectOrientation(self.handle, -1, orientation.tolist())
        self.simulation.wait()

    def get_position(self):
        position = self.simulation.sim.getObjectPosition(self.handle, -1)
        return position

    def get_orientation(self):
        orientation = self.simulation.sim.getObjectOrientation(self.handle, -1)
        return orientation

    def get_transform(self):
        """
        Returns a homogeneous transformation matrix
        """
        p = self.get_position()
        o = self.get_orientation()
        T = HomogeneousMatrix(Vector(p), Euler(o))
        return T

    def wait(self, steps=1):
        self.simulation.wait(steps=steps)


class Sphere(CoppeliaObject):
    def __init__(self, simulation):
        CoppeliaObject.__init__(self, simulation=simulation)

    def start(self, name='/Sphere'):
        handle = self.simulation.sim.getObject(name)
        self.handle = handle


class ReferenceFrame(CoppeliaObject):
    def __init__(self, simulation):
        CoppeliaObject.__init__(self, simulation=simulation)

    def start(self, name='/ReferenceFrame'):
        # Get the handles of the relevant objects
        handle = self.simulation.sim.getObject(name)
        self.handle = handle

    def show_target_point(self, target_position, target_orientation, wait_time=0.5):
        T = HomogeneousMatrix(target_position, target_orientation)
        self.set_position_and_orientation(T)
        self.simulation.wait_time(wait_time)

    def show_target_points(self, target_positions, target_orientations, wait_time=0.5):
        for i in range(len(target_positions)):
            self.show_target_point(target_position=target_positions[i],
                                   target_orientation=target_orientations[i],
                                   wait_time=wait_time)


class Cuboid(CoppeliaObject):
    def __init__(self, simulation):
        CoppeliaObject.__init__(self, simulation=simulation)

    def start(self, name='/Cuboid'):
        # Get the handles of the relevant objects
        handle = self.simulation.sim.getObject(name)
        self.handle = handle


class Dummy(CoppeliaObject):
    def __init__(self, simulation):
        CoppeliaObject.__init__(self, simulation=simulation)

    def start(self, name='/youBot/youBot_ref0'):
        # Get the handles of the relevant objects
        handle = self.simulation.sim.simxGetObjectHandle(name)
        self.handle = handle


def get_object_transform(simulation, base_name, piece_index):
    """
    Returns the position and orientation of th ith Cuboid in Coppelia
    (in the global reference frame)
    Used to get the transformation matrix of collection of objects with consecutive indices
    """
    obj = CoppeliaObject(simulation=simulation)
    if piece_index == 0:
        name = base_name
    else:
        name = base_name + '[' + str(piece_index) + ']'
    print('LOOKING FOR OBJECT TRANSFORM', name)
    obj.start(name)
    return obj.get_transform()
