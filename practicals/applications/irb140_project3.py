#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_project1.ttt scene before running this script.

The code beneath must be completed by the student in order to produce three different palletizations of the Red, Green and Blue pieces.

@Authors: Arturo Gil
@Time: April 2022
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


def find_color(robot, camera, frame, piece_index):
    """
    Places the camera on top of the piece.
    Saves the image for inspection.
    Computes the image to get the mean RGB value and the color name (red, green, blue).
    """
    robot.wait(80)
    T_piece = get_object_transform(simulation=robot.simulation, base_name='/Cuboid', piece_index=piece_index)
    # leer la posición de la pieza
    p_piece = T_piece.pos()
    # AQUÍ, to (target orientation), plantea una orientación para que, aproximadamente,
    # el robot vea correctamente la pieza
    to = Euler([np.pi / 2, -np.pi, 0])
    # position and orientation so that the camera sees the piece
    tp1 = p_piece + np.array([0.0, +0.1, 0.2])
    tp2 = p_piece + np.array([0.0, +0.1, 0.3])
    frame.show_target_points([tp1], [to])
    robot.moveJ(target_position=tp1, target_orientation=to, qdfactor=1.0)
    # captures an image and returns the closest color
    print('get_image')
    color = camera.get_color_name()
    robot.moveJ(target_position=tp2, target_orientation=to)
    return color


def pick(robot, gripper, frame, piece_index):
    """
    Picks the piece from the conveyor belt.
    """
    # Esta función devuelve una matriz de transformación de la pieza
    T_piece = get_object_transform(simulation=robot.simulation, base_name='/Cuboid', piece_index=piece_index)
    piece_length = 0.08
    tp1 = T_piece.pos() + np.array([0.0, 0.0, piece_length / 2])
    # orientación de la pieza. Se coge la primera de las dos solucioes posibles
    # (véase: conversión de una matriz R a ángulos de Euler).
    o_piece = T_piece.euler()[0]
    # ángulo de giro de la pieza sobre el eje Z (yaw)
    # yaw = o_piece.abg[2]

    # Se muestra, en esta línea, como plotear un target point en la escena
    # Importante: debemos especificar la posición y orientación de la ventosa sobre la pieza.
    frame.show_target_points([tp1], [o_piece])

    # robot.moveJ(target_position=tp1, target_orientation=o_piece, qdfactor=1.0)
    # ABRIR GRIPPER: en términos de VENTOSA IMPLICA APAGAR EL VACÍO.
    gripper.open()

    # EN ESTE PUNTO es necesario pensar en coger la pieza. Es importante pensar que la función get_object_transform devuelve
    # la posición y orientación del centro de masas de la pieza. Así pues, será necesario pensar en la manera de obtener
    # la posición y orientación de la cara del cubo que facilite su recogida.


    # CUANDO LA VENTOSA ESTÉ EXACTAMENTE SOBRE LA PIEZA, gripper.close la asirá (por succión).
    gripper.close()
    tp1 = T_piece.pos() + np.array([0.0, 0.0, 0.2])

    robot.moveJ(target_position=tp1, target_orientation=o_piece, qdfactor=1.0)


def place(robot, gripper, color, indices):
    # EN ESTE MOMENTO, EL ROBOT YA CONOCE:
    # A) El color de la pieza.
    # B) El índice de la pieza en el array. Nótese que, a partir del índice, es posible conocer la posición de la pieza
    # en el array. De esta manera, es posible mantener tres pallets independientes con tres colores independientes.
    if color == 'R':
        tp = Vector([-0.45, -0.6, 0.15])  # pallet R base position
        index = indices[0]
    elif color == 'G':
        tp = Vector([0.0, -0.6, 0.15])  # pallet R base position
        index = indices[1]
    else:
        tp = Vector([0.4, -0.6, 0.15])  # pallet R base position
        index = indices[2]
    # define que piece length and a small gap
    piece_length = 0.08
    piece_gap = 0.005
    # Con esto, el robot se mueve a una posición inicial
    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.moveAbsJ(q0, precision=True)

    # Resta, por tanto, calcular la posición y orientación en la que es necesario dejar
    # la pieza. Esto se hará en base a tp (target point) y especificando la orientacion deseada.
    # Nótese que, en este caso, los ejes de los pallets NO están alineados con los del robot. Por suerte, has estudiado
    # el tema 2 (herramientas matemáticas) y conoces formas de transformar las coordenadas desde un sistema de coordenadas a otro.

    # Cuando el robot haya llegado a la posición y orientación especificadas, se deberá soltar la pieza
    gripper.open(precision=True)


def pick_and_place():
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')
    camera = Camera(simulation=simulation)
    camera.start(name='/IRB140/RG2/camera')
    gripper = SuctionPad(simulation=simulation)
    gripper.start()
    # TCP DE LA VENTOSA!
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.105]), Euler([-np.pi / 2, 0, 0])))

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q_target=q0, precision=False)

    piece_index = 0
    color_indices = np.array([0, 0, 0])
    n_pieces = 30
    for i in range(n_pieces):
        print('PROCESSING PIECE: ', i)
        while True:
            if conveyor_sensor.is_activated():
                break
            simulation.wait()

        color = find_color(robot, camera, frame, piece_index)
        pick(robot, gripper, frame, piece_index)
        place(robot, gripper, color, color_indices)
        robot.moveAbsJ(q_target=q0, precision=False)

        # Next piece! Update indices
        piece_index += 1
        if color == 'R':
            color_indices[0] += 1
        elif color == 'G':
            color_indices[1] += 1
        else:
            color_indices[2] += 1

    simulation.stop()


if __name__ == "__main__":
    pick_and_place()

