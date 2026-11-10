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

# !/usr/bin/env python
# encoding: utf-8
"""
Aplicación de Sorting en CoppeliaSim con IRB140.
Fases: Detección en cinta, pesado (verificación), y sorting en cintas base.

@Authors: Arturo Gil / Modificado por Gemini
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

# !/usr/bin/env python
# encoding: utf-8
"""
Aplicación de sorting con cálculo de coordenadas globales y aproximación.
"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import ReferenceFrame
from robots.simulation import Simulation
from robots.camera import Camera


def barrido(robot, camera):
    # Orientación de la herramienta mirando hacia abajo
    to = Euler([0, np.pi, 0])
    # 1. POSICIÓN ELEVADA PARA OBSERVACIÓN GENERAL
    # Definimos un punto alto sobre la cinta para tener un campo de visión amplio
    pos_elevada = Vector([0.4, -0.2, 0.6])
    delta = 0.05
    for i in range(10):
        pos_elevada = pos_elevada + Vector([delta*i, 0, 0])
        print(f"Moviendo a posición elevada: {pos_elevada.pos()}")
        robot.moveJ(target_position=pos_elevada.pos(), target_orientation=to,
                    speed_factor=3.0, precision=False)
        id, Twa = detect_aruco(robot, camera)
        if id is not None:
            return id, Twa
    return None, None


def detect_aruco(robot, camera):
    # 2. DETECCIÓN ARUCO Y CÁLCULO DE COORDENADAS GLOBALES
    aruco_size = 0.015
    # robot.wait_time(0.5)  # Esperar a que la imagen se estabilice
    id_aruco, Tca = camera.detect_closer_aruco(show=True, aruco_size=aruco_size)

    if id_aruco is None:
        return None, None

    print(f"Pieza detectada (ID: {id_aruco}). Calculando posición global...")

    # Obtenemos la pose actual del robot (Mundo a Cámara/TCP)
    Twc = robot.directkinematics(robot.get_joint_positions())

    # Calculamos la matriz Mundo a ArUco: Twa = Twc * Tca
    # Esto transforma las coordenadas locales de la cámara a coordenadas de la escena
    Twa = Twc * Tca
    r

def sorting_operation():
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()

    # Iniciamos cámara y ventosa
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=25)
    camera.start(name='/IRB140/camera')
    suction = SuctionPad(simulation=simulation)
    suction.start()

    # TCP de la ventosa
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))

    barrido(robot=robot, camera=camera)



    # pos_global = Twa.pos()
    # print(f"Posición global de la marca ARUCO: {pos_global}")

    # 3. CÁLCULO DE PUNTO DE OBSERVACIÓN CERCANA
    # Definimos un punto exactamente sobre la ARUCO pero a menor altura (ej. 15cm por encima)
    distancia_observacion = 0.15
    pos_cercana = np.array([pos_global[0], pos_global[1], pos_global[2] + distancia_observacion])

    print(f"Moviendo a punto de observación cercana: {pos_cercana}")
    # Usamos moveL para una trayectoria lineal precisa hacia la pieza
    robot.moveL(target_position=pos_cercana, target_orientation=to, speed_factor=1.5, precision=True)

    # 4. (Opcional) Fase de pesado o recogida
    # Aquí podrías volver a leer el ArUco con más precisión antes de bajar del todo
    print("Observación cercana completada. Procediendo con la operación...")

    # Ejemplo de descenso final para recogida
    # robot.moveL(target_position=pos_global, target_orientation=to, speed_factor=1.0)
    # suction.suction_on()

    robot.wait_time(1.0)
    simulation.stop()


if __name__ == "__main__":
    sorting_operation()