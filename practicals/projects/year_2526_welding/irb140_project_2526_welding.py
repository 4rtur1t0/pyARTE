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
from practicals.projects.year_2526_welding.read_hershey import generate_text_trajectory
from robots.abbirb140 import RobotABBIRB140
from robots.objects import get_object_transform, ReferenceFrame
from robots.simulation import Simulation
from robots.camera import Camera
import matplotlib.pyplot as plt
from read_hershey import generate_text_trajectory


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


def engrave_text(robot, text, scale, transform, frame):
   """
   A completar por el estudiante.
   Debe:
   - generar la trayectoria para un texto.
   - transformar y escalar los puntos de la trayectoria.
   - la variable frame se emplea para visualizar la posición y orientación
   a la que se comanda el robot.
   - se debe comandar al robot a los puntos usando trayectorias rectas.

   Probad inicialmente con un texto de un par de letras solamente.
   """

   return


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

    # UNA PRUEBA PARA VER LA CINEMÁTICA DIRECTA (CONOCIDO EL TCP)
    # q = np.array([0, 0, 0, 0, 0, 0])
    # T = robot.directkinematics(q)
    # T.print_nice()
    # print(T.pos())
    # frame.show_target_point(target_position=T.pos(), target_orientation=T.R())

    # una prueba de soldadura
    # robot.moveJ(target_position=Vector([0.5, -.1, 0.15]),
    #             target_orientation=Euler([0, np.pi, 0]))
    # robot.moveL(target_position=Vector([0.5, 0.2, 0.15]),
    #             target_orientation=Euler([0, np.pi, 0]))

    # Encontramos la plancha
    q = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q)
    id, T0p = find_aruco_two_steps(robot, camera)
    frame.show_target_point(target_position=T0p.pos(), target_orientation=T0p.R())
    T0p.print_nice()

    # la función engrave_text esta función la debe completar el alumno/a
    # debe hacer todo: calcular las trayectorias del texto, transformar los puntos a un
    # sistema local y comandar al robot. Recibe como entrada una cadena de texto
    # y la debe escribir en el sistema de coordenadas indicado en transform con el escalado
    # deseado. La variable frame se emplea para mostrar la posición y orientación de los
    # puntos comandados
    engrave_text(robot=robot, text='dimito, estoy harto.', scale=0.04, transform=T0p, frame=frame)
    simulation.stop()


if __name__ == "__main__":
    engrave_on_plate()


