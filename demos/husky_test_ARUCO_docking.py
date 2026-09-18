#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Aarón Pastor Seguí, Arturo Gil
@Time: February 2024
"""
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


class Docking():
    def __init__(self):
        self.simulation = Simulation()
        self.simulation.start()
        # Connect to the robot
        self.robot = HuskyRobot(simulation=self.simulation)
        self.robot.start(base_name='/HUSKY')
        # A dummy object ath the robot center
        self.robot_center = CoppeliaObject(simulation=self.simulation)
        self.robot_center.start(name='/HuskyCenter')
        # an accelerometer
        self.accel = Accelerometer(simulation=self.simulation)
        self.accel.start(name='/Accelerometer')
        self.camera = Camera(simulation=self.simulation, resolution=1080, fov_degrees=60)
        self.camera.start(name='/camera')
        self.simulation.wait(steps=2)

        # ¿Trasformacón entre camara y centro de robot?
        self.Tcent = self.robot_center.get_transform()
        # Trasformación entre el centro del robot y el frontal donde se colocará el puerto magnetico
        self.Tfront = self.Tcent * HomogeneousMatrix(Vector([0.496, 0, 0]), Euler([np.pi, 0, -np.pi]))

    def observe_aruco(self):
        id, Tca = self.camera.detect_closer_aruco(show= False, aruco_size=0.1)
        return id, Tca

    def MovimientoSimple(self, MTAruco):
        #1º:Calculo de dirección de giro
        angulo = math.atan(MTAruco.pos()[0] / MTAruco.pos()[2]) #recordamos que la posicion 0 siempre es la X, la 1 la Y y la 2 la Z, pero el sistema de trabajo usa la Y como altura
        print("Aruco x", MTAruco.pos()[0], "Aruco y", MTAruco.pos()[2])
        if angulo<0:
            SentidoGiro=-1
        elif angulo>0:
            SentidoGiro=1
        else:
            SentidoGiro=0
        Giro = 0
        #2º:calculo de posición final deseada
        GiroDeseado=MTAruco.R()[2] + Giro
        self.robot.move(v=0, w=(0.5 * SentidoGiro))
        self.robot.wait()

    def MovimientoBasico(self):
        # for i in range(100):
        #     self.robot.move(v=2.0, w=0)
        #     self.robot.wait()
            # self.simulation.wait()
        #BUSQUEDA DE ARUCO
        #  TORQUES
        # now, obtain the mean torques or torques for each wheel
        # during 50 simulat steps.
        for i in range(300):
            # tau = robot.get_mean_wheel_torques()
            # axyz = accel.get_accel_data()
            id_aruco, Taruc = self.observe_aruco()


            #print(Tfront, "\nFrontal\nAruco")
            #print(Taruc)
            #print(Tcent, "a\n")
            #print (id_aruco)
            if Taruc is None:
                self.robot.move(v=0.5, w=0)
                self.robot.wait(1)
                self.robot.move(v=0, w=1.0)
                self.robot.wait(1)

                #for j in range(360):
                #    robot.move(v=0, w=0.1)
                    #print(Tfront, "\n")
                #    robot.wait()
                print("Aruco no encontrado aún\n")

            else:
                Matrizdireccion = Taruc * self.Tfront
                print("Aruco encontrado\n")
                self.robot.move(v=1.0, w=0)
                self.robot.wait(1)
                #print(Taruc,"\n\nAruco nº:",id_aruco)
                #Direccion= Matrizdireccion.pos()
                #print("Direccion \n\n", Direccion)
            # self.robot.wait()
        #MovimientoSimple(robot=robot, MTArucfrontIn=Matrizdireccion)
        SentidoGiro=0
        self.simulation.stop()


if __name__ == "__main__":
    docking = Docking()

    docking.MovimientoBasico()
