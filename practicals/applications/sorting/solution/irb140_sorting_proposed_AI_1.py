#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_sorting.ttt scene before running this script.

The code beneath must be completed by the student in order to produce a piece sorting application. Place the different
pieces on the 4 conveyor belts.

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


def initial_pose(robot, suction):
    # desactivamos la ventosa (no aspira)
    suction.suction_off()
    tp1 = Vector([0.5, 0.0, 0.7])
    to1 = Euler([0, np.pi, 0])
    robot.moveJ(target_position=tp1, target_orientation=to1, speed_factor=3.0)



def sorting_operation():
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()

    # Configuración de cámara y ventosa
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=45)
    camera.start(name='/IRB140/camera')
    suction = SuctionPad(simulation=simulation)
    suction.start()

    # Definir el TCP: el extremo de la ventosa
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))

    conveyor_sensor = ProxSensor(simulation=simulation)
    conveyor_sensor.start(name='/conveyor/prox_sensor')

    # Posiciones de descarga (bins) según el ID de ArUco
    # Ajusta estas coordenadas según la ubicación de tus 4 rampas
    bins = {
        1: Vector([0.3, 0.5, 0.3]),  # Ejemplo: ID 1 a cinta 1
        2: Vector([0.3, -0.5, 0.3]),  # Ejemplo: ID 2 a cinta 2
        3: Vector([-0.3, 0.5, 0.3]),
        4: Vector([-0.3, -0.5, 0.3])
    }

    # Posición de reposo/escaneo
    q_home = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot.moveAbsJ(q_target=q_home, precision=False, speed_factor=3.0)

    while True:
        robot.moveAbsJ(q_target=q_home, speed_factor=3.0)
        initial_pose(robot, suction)
        # 1. Esperar a que el sensor de la cinta detecte una pieza
        if conveyor_sensor.is_activated():
            print("Pieza detectada en cinta. Iniciando búsqueda ArUco...")

            # 2. Detectar marca ArUco
            aruco_size = 0.02  # Ajustar al tamaño real en metros
            id, Tca = camera.detect_closer_aruco(show=False, aruco_size=aruco_size)

            if id is not None:
                print(f'Pieza encontrada! ID: {id}')

                # 3. Calcular posición de la pieza en el mundo
                # Twc = camera.get_pose()  # Pose de la cámara en el mundo
                # Twa = Twc * Tca  # Pose de la marca en el mundo
                q = robot.get_joint_positions()
                Te = robot.directkinematics(q)
                # transformacion ventosa--> cámara
                Tec = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, -np.pi / 2]))
                # transformacion total
                Twa = Te * Tec * Tca

                target_pos = Twa.pos()
                # Forzamos una orientación vertical para el agarre (ventosa mirando abajo)
                target_orient = Euler([0, np.pi, 0])

                # 4. Movimiento de aproximación (0.1m por encima)
                approach_pos = target_pos + np.array([0, 0, 0.1])
                robot.moveJ(target_position=approach_pos, target_orientation=target_orient, speed_factor=2.0)

                # 5. Pick (Bajar y activar succión)
                robot.moveL(target_position=target_pos, target_orientation=target_orient, precision=True)
                suction.suction_on()
                # robot.wait_time(0.5)

                # 6. Retraer y mover al contenedor correspondiente
                robot.moveL(target_position=approach_pos, target_orientation=target_orient)

                drop_pos = bins.get(id, Vector([0, -0.4, 0.3]))  # Default si no está en el dict
                robot.moveJ(target_position=drop_pos.pos(), target_orientation=target_orient)

                # 7. Soltar pieza
                suction.suction_off()
                # robot.wait_time(0.2)

                # Volver a posición de espera
                # robot.moveAbsJ(q_target=q_home)
            else:
                print("No se pudo identificar el ArUco de la pieza.")

        # robot.wait_time(0.1)  # Evitar saturar la CPU


if __name__ == "__main__":
    sorting_operation()


