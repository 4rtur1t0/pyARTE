#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Aarón Pastor Seguí, Arturo Gil
@Time: February 2024
"""
import math

import time

from asyncio import wait_for
from time import sleep

import numpy as np
from numpy.f2py.crackfortran import endifs

from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.vector import Vector
from robots.accelerometer import Accelerometer
from robots.objects import CoppeliaObject
from robots.ouster import Ouster
from robots.simulation import Simulation
from robots.Husky import HuskyRobot
from robots.camera import Camera

def observe_aruco(camera):
    id, Tca = camera.detect_closer_aruco(show= False, aruco_size=0.1)
    return id, Tca


def movimiento_simple(MTArucfrontIn):
    #angulo= arcotang(seno=y/coseno=x)
    # angulo = math.atan(abs(MTArucfrontIn.pos()[0])/ abs(MTArucfrontIn.pos()[1]))
    angulo = np.arctan2(MTArucfrontIn.pos()[2], MTArucfrontIn.pos()[1])
    v = 0.1
    w = 0.1*angulo
    # id_aruco, MTArucfrontActual = observe_aruco(camera=camera)
    # cuando realiza resta de angulos tiene en cuenta que 300º=-60º
    # Giro=MTArucfrontIn.R()[3] - MTArucfrontActual.R()[3]
    # for Giro != angulo
    #     if MTArucfrontIn.R()[3]-MTArucfrontActual.R()[3] <= 0
    #         robot.move(v=0,w=0.5)
    #         robot.wait
    #     else
    #         robot.move(v=0, w=0.5)
    #         robot.wait
    return v, w



def calcula_accion(robot, Taruc):

    # Tfront = Tcent * HomogeneousMatrix(Vector([0.496, 0, 0]), Euler([np.pi, 0, -np.pi]))
    if Taruc is None:
        # robot.move(v=0, w=1)

        # robot.move(v=0.2, w=0.5)
        # robot.wait()
        v = 0.2
        w = 0.5

        # for j in range(360):
        #    robot.move(v=0, w=0.1)
        # print(Tfront, "\n")
        #    robot.wait()
        print("Aruco no encontrado aún\n")

    else:
        # debe depender de de las transformaciones
        Matrizdireccion = Taruc # * Tfront
        print("Aruco encontrado\n")
        # robot.move(v=0, w=0)
        v, w = movimiento_simple(Taruc)
    return v, w






def simulate():
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # A dummy object ath the robot center
    robot_center = CoppeliaObject(simulation=simulation)
    robot_center.start(name='/HuskyCenter')
    # an accelerometer
    accel = Accelerometer(simulation=simulation)
    accel.start(name='/Accelerometer')
    camera = Camera(simulation=simulation, resolution=1080, fov_degrees=60)
    camera.start(name='/camera')
    simulation.wait(steps=2)

    # ¿Trasformacón entre camara y centro de robot?

    # Trasformación entre el centro del robot y el frontal donde se colocará el puerto magnetico



    #BUSQUEDA DE ARUCO
    #  TORQUES
    # now, obtain the mean torques or torques for each wheel
    # during 50 simulat steps.
    for i in range(150):
        # tau = robot.get_mean_wheel_torques()
        # axyz = accel.get_accel_data()
        id_aruco, Taruc = observe_aruco(camera=camera)
        v, w = calcula_accion(robot=robot, Taruc=Taruc)
        robot.move(v=v, w=w)
        robot.wait()
        #print(Tfront, "\nFrontal\nAruco")
        #print(Taruc)
        #print(Tcent, "a\n")
        #print (id_aruco)

        # robot.wait()

        # print(T)
        # print(tau)
        # print(axyz)

    #MovimientoSimple(robot=robot, MTArucfrontIn=Matrizdireccion)

    simulation.stop()


if __name__ == "__main__":
    simulate()

# def MovimientoSimple(robot, MTArucfront,):
#     coordobj = Matrizobjetivo.pos()
#     print(coordobj)
#     coordsalida = Matrizpartida.pos()
#     print(coordsalida)
#     #En caso de que los valores de la posicion se encuentres referidos al origen del mundo
#     angulo = math.tan(abs(Matrizpartida.pos()[0]-Matrizobjetivo.pos()[0])/abs(Matrizpartida.pos()[1]-Matrizobjetivo.pos()[1]))
#     #angsalida=Matrizpartida.R()
#     #angobj=Matrizobjetivo.R()
#     matrizangsalida1=Matrizpartida*HomogeneousMatrix(Vector([0,0,0]),Euler([0,0,angulo]))
#     #angsalida1=matrizangsalida1.R()
#     while Matrizpartida.R[2] != matrizangsalida1.R[2]:
#         robot.move(v=0, w=1)
#         # Matrizpartida=
#     Distancia=math.sqrt((abs(Matrizpartida.pos(1)-Matrizobjetivo.pos(1)))**2+(abs(Matrizpartida.pos(2)-Matrizobjetivo.pos(2)))**2)
#     for i in range(Distancia):
#         robot.move(v=1,w=0)
#     # while
#     return