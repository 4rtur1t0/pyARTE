#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_project_ARUCO_2526.ttt scene before running this script.

The code beneath must be completed by the student in order to engrave/solder letters on
 a metal plate.

@Authors: Arturo Gil
@Time: November 2025
"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from practicals.projects.year_2526_welding.solution.read_hershey import generate_text_trajectory
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.objects import ReferenceFrame
from robots.simulation import Simulation
from robots.camera import Camera
from hershey import hershey_dict


def find_aruco_two_steps(robot, camera):
    id, T0p = look_for_aruco(robot, camera, show=True, aruco_size=0.03)

    robot.moveJ(target_position=T0p.pos() + np.array([0.0, 0.05, 0.05]),
                target_orientation=Euler([0, np.pi, 0]))
    id, T0p = look_for_aruco(robot, camera, show=True, aruco_size=0.03)
    return id, T0p


def look_for_aruco(robot, camera, show=True, aruco_size=0.03):
    """
    In case we want a list to all ARUCOS along with its transformation
    """
    id, Tca = camera.detect_closer_aruco(show=show, aruco_size=aruco_size)
    if id is None:
        print('could not find piece')
        return None, None
    q = robot.get_joint_positions()
    Te = robot.directkinematics(q)
    # transformacion tcp--> cámara
    Tec = HomogeneousMatrix([[ 0.,       1.,       0. ,      0.],
                             [-1.,       0.,       0.,      -0.05],
                             [ 0.,       0.,       1.,      -0.15],
                             [ 0.,       0.,       0.,       1.]])
    # transformacion total
    T = Te * Tec * Tca
    return id, T


def calibrate(robot, camera):
    """
    Must be used to compute the transformation between the suction cap and the camera.
    The position and orientation of the reference system captured by the camera on the ARUCO must be known.
    Place the ARUCO on a known position and orientation and make the camera look at it from a clear vantage point.
[[ 0.       1.       0.       0.     ]
 [-1.       0.       0.      -0.05002]
 [ 0.       0.       1.      -0.09522]
 [ 0.       0.       0.       1.     ]]

    """
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    robot.moveJ(target_position=Vector([0.5, -0.2, 0.3]),
                target_orientation=Euler([0, np.pi, 0]))
    id, T = find_aruco_two_steps(robot=robot, camera=camera)
    id, Tca = camera.detect_closer_aruco(show=True, aruco_size=0.03)
    print('Tca')
    Tca.print_nice()
    q = robot.get_joint_positions()
    T = robot.directkinematics(q)
    # KNOWN POSITION/ORIENTATION OF THE ARUCO
    T0A = HomogeneousMatrix(Vector([0.5, -0.2, 0.15]), Euler([0, 0, 0]))
    # obtain the relative transformation
    Ttcp_cam = T.inv()*T0A*Tca.inv()
    Ttcp_cam.print_nice()
    return Ttcp_cam


def engrave_text(robot, text, scale, transform, frame):
    Tab = HomogeneousMatrix(Vector([0.25, 0.4, 0]), Euler([0, 0, -np.pi/2]))
    T = transform*Tab
    orient = Euler([0, np.pi, 0])
    robot_trajectory = generate_text_trajectory(text=text)
    for letter_trajectory in robot_trajectory:
        for point in letter_trajectory:
            vrel = Vector([scale * point[0], scale * point[1], point[2], 1])
            vglobal = T*vrel
            # frame.show_target_point(target_position=vglobal.pos(), target_orientation=orient)
            robot.moveL(target_position=vglobal.array[0:3],
                        target_orientation=orient, speed_factor=1.0)


def engrave_on_plate():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=45)
    camera.start(name='/IRB140/camera')

    # TCP DE LA HERRAMIENTA!
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.2]), Euler([0, 0, 0])))
    # Ttcp_c=calibrate(robot, camera)

    # Encontramos la plancha
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    id, T0p = find_aruco_two_steps(robot, camera)
    frame.show_target_point(target_position=T0p.pos(), target_orientation=T0p.R())
    T0p.print_nice()
    engrave_text(robot=robot, text='dimito, estoy harto.', scale=0.04, transform=T0p, frame=frame)
    simulation.stop()


if __name__ == "__main__":
    engrave_on_plate()


