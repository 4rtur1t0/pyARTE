#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_sorting.ttt scene before running this script.

The code beneath sorts a number of pieces (identified with ARUCO markers) in different conveyor belts.

@Authors: Arturo Gil
@Time: March 2026
"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import ReferenceFrame
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation
from robots.camera import Camera


def detect_ARUCO_transform(robot, camera, show=True, aruco_size=0.03):
    """
    In case we want a list to all ARUCOS along with its transformation
    """
    id, Tca = camera.detect_closer_aruco(show=show, aruco_size=aruco_size)
    if id is None:
        print('could not find piece')
        return None, None
    q = robot.get_joint_positions()
    Te = robot.directkinematics(q)
    # transformacion ventosa--> cámara
    Tec = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, -np.pi / 2]))
    # transformacion total
    T = Te * Tec * Tca
    return id, T


def wait_piece(robot, conveyor_sensor):
    # ESPERAMOS A QUE LA PIEZA CORTE EL SENSOR FOTOELÉCTRICO Y SE PARE LA CINTA TRANSPORTADORA
    while True:
        if conveyor_sensor.is_activated():
            break
        robot.wait()


def detect_arucos(robot, camera, area):
    if area == 'conveyor':
        to = Euler([0, np.pi, 0])
        tp = Vector([0.4, -0.25, 0.55])
    elif area == 'weighing':
        to = Euler([0, np.pi, 0])
        tp = Vector([-0.4, -0.35, 0.65])

    delta = 0.05
    for j in range(6):
        for i in range(6):
            robot.moveJ(target_position=tp+Vector([i*delta, -j*delta, 0]),
                        target_orientation=to, speed_factor=3.0)
            id, Tpick = detect_two_stages(robot, camera)
            if id is not None:
                return id, Tpick
    return None, None


def detect_two_stages(robot, camera):
    # a continuación, se calcula la transformación total T de la marca
    aruco_size = 0.015
    id, T = detect_ARUCO_transform(robot=robot, camera=camera, show=True, aruco_size=aruco_size)
    if id is None:
        return None, None
    # el eje Z de la marca apunta hacia fuera, se necesita una transformación relativa para poderla asir
    Trel = HomogeneousMatrix(Vector([0, 0, 0]), Euler([0, np.pi, 0]))
    # robot.frame.show_target_point(target_position=T.pos(), target_orientation=T.R())
    Tao = HomogeneousMatrix(Vector([0, 0, -0.1]), Euler([0, 0, 0]))
    Tobs = T*Trel*Tao
    Tec = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, 0]))
    Trobot = Tobs*Tec.inv()
    # now make a second observation on top of the ARUCO, but at a fixed orientation
    robot.moveJ(target_position=Trobot.pos(), target_orientation=Trobot.R(),
                speed_factor=3.0, precision=True)
    id, T = detect_ARUCO_transform(robot=robot, camera=camera, show=False, aruco_size=aruco_size)
    if id is None:
        return None, None
    Tpick = T * Trel
    return id, Tpick

def pick(robot, suction, Tpick):
    # desactivamos la ventosa (no aspira)
    suction.suction_off()
    # se coloca la ventosa sobre la marca ARUCO
    # robot.moveJ(target_position=Tpick.pos() + np.array([0, 0, 0.1]), target_orientation=Tpick.R(), speed_factor=3.0)
    robot.moveL(target_position=Tpick.pos(), target_orientation=Tpick.R(), speed_factor=2.0)
    # se activa la succión
    suction.suction_on()
    # salimos hacia arriba (se suma un vector relativo!)
    robot.moveL(target_position=Tpick.pos() + np.array([0, 0, 0.2]), target_orientation=Tpick.R(), speed_factor=2.0)

def place_conveyors(robot, suction, id):
    if id == 0:
        tp = Vector([0.1, -0.65, 0.12])
        to = Euler([0, np.pi, 0])
    elif id == 2:
        tp = Vector([0.1, -0.42, 0.155])
        to = Euler([3*np.pi/4, 0, -np.pi/2])
    elif id == 3:
        tp = Vector([0.1, 0.45, 0.15])
        to = Euler([0, np.pi, -np.pi/2])
    elif id == 4:
        tp = Vector([0.1, 0.65, 0.15])
        to = Euler([0, np.pi, -np.pi/2])
    else:
        tp = Vector([0.1, 0.6, 0.15])
        to = Euler([0, np.pi, 0])
    robot.moveJ(target_position=tp+Vector([0, 0, 0.3]), target_orientation=to, speed_factor=3.0, precision=False)
    robot.moveJ(target_position=tp, target_orientation=to, speed_factor=3.0)
    # se desactiva la ventosa
    suction.suction_off()
    robot.moveJ(target_position=tp + Vector([0, 0, 0.3]), target_orientation=to, speed_factor=3.0, precision=False)
    # reset, new start of the process
    q = np.array([-np.pi/8, 0, 0, 0, np.pi/2, 0])
    robot.moveAbsJ(q_target=q, precision=False, speed_factor=3.0)

def place_weight(robot, suction, id):
    tp = Vector([-0.4, -0.4, 0.4])
    if id == 0:
        to = Euler([0, np.pi, 0])
    elif id == 2:
        to = Euler([3*np.pi/4, 0, -np.pi/2])
    elif id == 3:
        to = Euler([0, np.pi, -np.pi/2])
    elif id == 4:
        to = Euler([0, np.pi, -np.pi/2])
    else:
        to = Euler([0, np.pi, 0])
    robot.moveJ(target_position=tp+Vector([0, 0, 0.3]), target_orientation=to, speed_factor=3.0, precision=False)
    robot.moveJ(target_position=tp, target_orientation=to, speed_factor=3.0)
    # se desactiva la ventosa
    suction.suction_off()
    robot.moveJ(target_position=tp + Vector([0, 0, 0.3]), target_orientation=to, speed_factor=3.0, precision=False)

def sorting_operation():
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=25)
    camera.start(name='/IRB140/camera')
    suction = SuctionPad(simulation=simulation)
    suction.start()
    # TCP DE LA VENTOSA! OJO!: debe ser el adecuado para la escena
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))
    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')

    # Se mueve el robot a una posición articular q inicial
    q = np.array([0, 0, 0, 0, np.pi/2, 0])
    robot.moveAbsJ(q_target=q, precision=False, speed_factor=3.0)

    for i in range(17):
        wait_piece(robot, conveyor_sensor)
        id, Tpick = detect_arucos(robot, camera, 'conveyor')
        if id is None:
            continue
        # pick, as computed
        pick(robot, suction, Tpick)
        # place on weight area
        place_weight(robot, suction, id)
        # detect on weighing area
        id, Tpick = detect_arucos(robot, camera, 'weighing')
        if id is None:
            continue
        # pick again, as computed
        pick(robot, suction, Tpick)
        place_conveyors(robot, suction, id)
    simulation.stop()


if __name__ == "__main__":
    sorting_operation()


