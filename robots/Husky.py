#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez
@Time: February 2024
"""
from robots.robot import Robot
import numpy as np
import yaml


class HuskyRobot(Robot):
    def __init__ (self, simulation):
        Robot.__init__(self, simulation=simulation)

    def start (self, base_name='/HUSKY'):
        wheelRL = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointRLW')
        wheelFL = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointFLW')
        wheelRR = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointRRW')
        wheelFR = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointFRW')
        wheeljoints = []
        wheeljoints.append(wheelRL)
        wheeljoints.append(wheelFL)
        wheeljoints.append(wheelRR)
        wheeljoints.append(wheelFR)
        self.joints = wheeljoints
        self.width = 0.555
        self.wheel_radius = 0.165
        self.Vmax = 0.5
        self.Wmax = np.pi / 4
        self.Vmin = 0.1
        self.amax = 0.1
        self.alphamax = 0.2
        self.V = 0
        self.W = 0

    def calcVW (self):
        """
        Calculate linar and angular velocity of the robot from the angular velocities of the wheels.

        :return: Linear velocity, angular velocity
        """
        [wl, _, wr, _] = self.get_joint_speeds()
        V = (self.wheel_radius * (wr + wl)) / 2

        W = (self.wheel_radius * (wr - wl)) / self.width
        return V, W

    def move (self, v, w):
        """

        :param v: Desired linear velocity
        :param w: Desired angular velocity

        """
        v, w = self.accelcontrol(v, w)
        self.V = v
        self.W = w
        r = self.wheel_radius
        b = self.width
        wl = (v - w * (b / 2)) / r
        wr = (v + w * (b / 2)) / r

        self.simulation.sim.setJointTargetVelocity(self.joints[0], wl)
        self.simulation.sim.setJointTargetVelocity(self.joints[1], wl)
        self.simulation.sim.setJointTargetVelocity(self.joints[2], wr)
        self.simulation.sim.setJointTargetVelocity(self.joints[3], wr)

    def getParams (self):
        """
        Get robot parameters from a config file.

        """
        try:
            with open(r'/config/Husky_Config.yaml') as file:
                param_list = yaml.load(file, Loader=yaml.FullLoader)
                print(param_list)
        except:
            print("YAML loading error!...")
        try:
            self.Vmax = param_list.get('Vmax')
            self.Wmax = param_list.get('Wmax')
            self.Vmin = param_list.get('Vmin')
            self.amax = param_list.get('amax')
            self.alphamax = param_list.get('alphamax')
            self.Vmax = float(self.Vmax)
            self.Wmax = float(self.Wmax)
            self.Vmin = float(self.Vmin)
            self.amax = float(self.amax)
            self.alphamax = float(self.alphamax)

        except:
            print("Error getting params from config.YAML!...")

    def checkmax (self, V, w):
        """
        :param V: Linear velocity
        :param w: Angular velocity
        """
        Vout = np.clip(V, self.Vmin, self.Vmax)
        wout = np.clip(w, -self.Wmax, self.Wmax)
        return Vout, wout

    def accelcontrol (self, Vcom, Wcom):

        delta_time = 0.05
        a = (Vcom - self.V) / delta_time
        ar = np.clip(a, -self.amax, self.amax)
        v = delta_time * ar + self.V
        alpha = (Wcom - self.W) / delta_time
        alphar = np.clip(alpha, -self.alphamax, self.alphamax)
        w = delta_time * alphar + self.W
        return v, w

    def goto_objective (self, base, puntos, velocity, skip):
        """

        :param base: Base of robot
        :param puntos: Points to follow
        :param velocity:
        :param skip: True: If we are closer to the next point than the current point we jump into the next one.

        """
        Kv = 2
        Kw = 1  # 0.4
        Vdes = velocity
        i = 0
        for punto in puntos:
            xdes = punto[0]
            ydes = punto[1]
            i += 1
            while (np.linalg.norm(punto - base.get_transform().pos())) > 0.5:
                print('Distancia al siguiente punto:', (np.linalg.norm(punto - base.get_transform().pos())))
                T_base = base.get_transform()
                om = T_base.euler()[0].abg[2]
                pos = T_base.pos()
                omdes = np.arctan2(ydes - pos[1], xdes - pos[0])
                eom = omdes - om
                if skip:
                    if i == len(puntos) - 1: i = 0
                    if (np.linalg.norm(puntos[i + 1] - pos)) < (np.linalg.norm(punto - pos)):
                        break
                    if abs(eom) > np.pi / 2.5 and np.linalg.norm(punto - base.get_transform().pos()) < 2:
                        break
                w = Kv * eom
                w = np.clip(w, -self.Wmax, self.Wmax)
                wcont = abs(w / self.Wmax)
                V = Vdes * (1 - Kw * wcont ** 2)
                V, w = self.checkmax(V, w)
                self.move(V, w)
                self.wait()

#  Añadir a pyArte en robots.objects
# def getVelocity(self):
#      linear_velocity = []
#      angular_velocity = []
#      linear_velocity,angular_velocity = self.simulation.sim.getObjectVelocity(self.handle)
#      return linear_velocity,angular_velocity
