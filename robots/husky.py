#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez
@Time: February 2024
"""
from robots.robot import Robot
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix

from artelib.rotationmatrix import RotationMatrix


class HuskyRobot(Robot):
    def __init__ (self, simulation):
        Robot.__init__(self, simulation=simulation)

    def start (self, base_name='/HUSKY'):
        robotbase = self.simulation.sim.getObject(base_name)
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
        self.radius = 0.165


    def mov_orientacion (self, base, objetivo):
        T_base = base.get_transform()
        orientacion = T_base.euler()[0]
        final = orientacion.abg[2] + objetivo
        print('ver aqui')
        print(final)
        while True:
            T_base = base.get_transform()
            orientacion = T_base.euler()[0]
            print('-------------')
            print(orientacion.abg[2])
            e = np.linalg.norm(final - orientacion.abg[2])
            print(e)
            if e < 0.01:
                self.move(v=0, w=0)
                break
            if (objetivo) > np.pi:
                self.move(v=0, w=-0.05)
            elif objetivo < np.pi and objetivo >= 0:
                self.move(v=0, w=0.05)
            elif objetivo > -np.pi and objetivo < 0:
                self.move(v=0, w=-0.05)
            else:
                self.move(v=0, w=0.05)
            self.wait()

    def mov_posicion (self, base, objetivo):
        while True:
            self.move(v=0.2, w=0)
            T_base = base.get_transform()
            posicion = T_base.pos()
            print('posac')
            print(posicion)
            e = np.linalg.norm(objetivo - posicion)
            print(e)
            if e < 0.25:
                self.move(v=0, w=0)
                break
            self.wait()

    def sigue_puntos(self, base, puntos):
        print(len(puntos))
        for i in range(len(puntos)):
            if i == 0:
                angulo = -np.arctan2(puntos[i][1], puntos[i][0])
                print('angulo a girar')
                print(angulo)
                self.mov_orientacion(base, angulo)
                self.mov_posicion(base=base, objetivo=puntos[i])
            else:
                print(puntos[i])
                T_base = base.get_transform()
                Tfinal = HomogeneousMatrix((puntos[i]), Euler([0, 0, 0]))
                Tinicio = HomogeneousMatrix(T_base.pos(), Euler([0, 0, 0]))
                tx = Tinicio.inv() * Tfinal
                print('---------------------Tfinal---------------------------')
                print(Tfinal)
                print('---------------------Tinicio---------------------------')
                print(Tinicio)
                print('---------------------TX--------------------------')
                print(tx)
                print('-------------------------------------------------------')
                print(np.arctan2(tx.pos()[1], tx.pos()[0]))
                print(T_base.euler()[0].abg[2])
                angulo = np.arctan2(tx.pos()[1], tx.pos()[0]) - T_base.euler()[0].abg[2]
                print('angulo a girar')
                print(angulo)
                self.mov_orientacion(base, angulo)
                self.mov_posicion(base=base, objetivo=puntos[i])

    # def path_planning(self):
    #     x=0
    #     y=0
    #     om=0
    #
    #     q= Vector([x, y, om]).T()
    #     R=0.165
    #     b = 0.4207/2
    #     v=(R*(wl + wr))/2
    #     w=(R*(wr -wl))/(2*b)
    #     vx=v*np.cos(om)
    #     vy=v*np.sen(om)
    #     vom=w
    #     vq= Vector([vx, vy, vom]).T()
    #     u=Vector([wl,wr]).T()
    #
    #     e=0.5
    #     xc= x +e*np.cos(om)
    #     yc = y + e * np.sen(om)
    #     n=Vector([xc,yc]).T()
    #     vxc = x -om* e * np.cos(om)
    #     vyc = y + om*e * np.sen(om)
    #     vn = Vector([vxc, vyc]).T()
    #     jacob=(R/2)*np.array([(np.cos(om)+(e/b)*np.sin(om),np.cos(om)-(e/b)*np.sin(om)),
    #                           (np.sin(om)+(e/b)*np.cos(om),np.sin(om)-(e/b)*np.cos(om))])
    #     vn=jacob*u
    #     jacobinv=(1/R)*np.array([(np.cos(om)+(b/e)*np.sin(om),np.sin(om)-(b/e)*np.cos(om)),
    #                           (np.cos(om)+(b/e)*np.sin(om),np.sin(om)-(b/e)*np.cos(om))])
    #     u=jacobinv*vn

    # def path_planning_line (self,base,obj):
    #     T_base = base.get_transform()
    #     pA=T_base.pos()
    #     pB = obj
    #     oA = T_base.euler()[0]
    #     oB = oA
    #     n1 = n_movements_pos(pB, pA)
    #     n2 = n_movements_orient(oB, oA)
    #     n = max(n1, n2)
    #     t = np.linspace(0, 1, n)
    #     a=0.5
    #     x=np.zeros(n+1)
    #     y = np.zeros(n + 1)
    #     phi = np.zeros(n + 1)
    #
    #     x[0] =T_base.pos()[0]
    #     y[0] = T_base.pos()[1]
    #     phi[0] = T_base.euler()[0].abg[2]
    #
    #     hx = np.zeros(n + 1)
    #     hy = np.zeros(n + 1)
    #
    #     hx[0]= x[0]+ a*np.cos(T_base.euler()[0].abg[2])
    #     hy[0]= y[0] + a * np.sin(T_base.euler()[0].abg[2])
    #
    #
    #     hxd= 2*np.cos(t)
    #     hyd = 2 * np.sin(t)
    #
    #     hxdp= -2*np.sin(t)
    #     hydp = 2 * np.cos(t)
    #
    #     uRef=np.zeros(n)
    #     wRef = np.zeros(n)
    #
    #     hxe=np.zeros(n)
    #     hye = np.zeros(n)
    #
    #     for k in range(n):
    #
    #         hxe[k]=hxd[k]-hx[k]
    #         hye[k] = hyd[k] - hy[k]
    #
    #         he= np.array([[hxe[k] ],[hye[k] ]])
    #
    #         J=np.array([[np.cos(phi[k]), -a*np.sin(phi[k])],
    #                     [np.sin(phi[k]), a*np.cos(phi[k])]])
    #
    #         K=np.array([[1 ,0],[0, 1]])
    #
    #         hdp= np.array([[hxdp[k]],[hydp[k]]])
    #
    #         qpRef = np.linalg.pinv(J)@(hdp+K@he)
    #
    #         uRef[k]= qpRef[0][0]
    #         wRef[k]= qpRef[1][0]
    #
    #         phi[k+1]=phi[k]+ (1/n)*wRef[k]
    #
    #         xp= uRef[k]*np.cos(phi[k+1])
    #         yp = uRef[k] * np.sin(phi[k + 1])
    #
    #         x[k + 1] = x[k] + (1 / n) * xp
    #         y[k + 1] = y[k] + (1 / n) * yp
    #
    #         hx= x[k+1]+a*np.cos(phi[k+1])
    #         hy = y[k + 1] - a * np.sin(phi[k + 1])

    def chackmax (self, vmax, wmax, wr, wl):
        change = False

        vaux = (self.radius * (wr + wl)) / 2
        print('vaux',vaux)

        waux = (self.radius * (wr - wl)) / self.width
        print('waux', waux)
        if vaux > vmax:
            vaux = vmax
            change = True
        elif vaux<-vmax:
            vaux=-vmax
            change = True
        if waux > wmax:
            waux = wmax
            change = True
        elif waux<-wmax:
            waux=-wmax
            change = True
        if change:
            wr = (vaux + waux * (self.width / 2)) / self.radius
            wl = (vaux - waux * (self.width / 2)) / self.radius

        return wl, wr


    def calcVW(self,wl,wr):
        vaux = (self.wheel_radius * (wr + wl)) / 2
        # print('vaux', vaux)

        waux = (self.radius * (wr - wl)) / self.width
        # print('waux', waux)
        return vaux,waux

    def goto (self, base, puntos):
        e = 0.1
        Kp=1.2
        # Kp = np.array([(0.5, 0), (0, 0.5)])
        Kd = 0.5 #np.array([(0.1, 0), (0, 0.1)])  # 0.3
        b = self.width / 2
        V = 1
        Vmax = 1
        Wmax = np.pi
        for punto in puntos:
            T_base = base.get_transform()
            Tfinal = HomogeneousMatrix((punto), Euler([0, 0, 0]))
            Tinicio = HomogeneousMatrix(T_base.pos(), Euler([0, 0, 0]))
            tx = Tinicio.inv() * Tfinal
            print('angulo base', T_base.euler()[0].abg[2])
            print('angulo punto', np.arctan2(tx.pos()[1], tx.pos()[0]))
            angulo = np.arctan2(tx.pos()[1], tx.pos()[0]) - T_base.euler()[0].abg[2]
            print('angulo a llegar', -angulo)
            nref = np.array([(punto[0] + e * np.cos(-angulo)), (punto[1] + e * np.sin(-angulo))])
            print('nref', nref)

            [wl, nada, wr, nada2] = self.get_joint_speeds()
            Va, Wa = self.calcVW(wr=wr, wl=wl)

            vnref = np.array([(1), (1)])



            while (np.linalg.norm(punto - base.get_transform().pos())) > 0.7:

                T_base = base.get_transform()
                om = T_base.euler()[0].abg[2]
                pos = T_base.pos()
                jacobinv =  np.array(
                    [(np.cos(om) + (b / e) * np.sin(om), np.sin(om) - (b / e) * np.cos(om)),
                     (np.cos(om) - (b / e) * np.sin(om), np.sin(om) + (b / e) * np.cos(om))])

                n = np.array([(pos[0] + e * np.cos(om)), pos[1] + e * np.sin(om)])
                print('n', n)
                [wl, nada, wr, nada2] = self.get_joint_speeds()
                print('wl antes', wl, 'wr antes', wr)
                w = (self.radius * (wr - wl)) / self.width
                vn=np.array([(pos[0]-w*e*np.sin(om)),pos[1]+e*w*np.cos(om)])

                u = jacobinv @ ( Kd*(vnref - vn)+Kp * (nref - n))
                wl = u[0]
                wr = u[1]
                # wl, wr = self.chackmax(Vmax, Wmax, wr, wl)
                print('wl despues', wl, 'wr despues', wr)
                print('error', np.linalg.norm(punto - base.get_transform().pos()))

                self.simulation.sim.setJointTargetVelocity(self.joints[0], wl)
                self.simulation.sim.setJointTargetVelocity(self.joints[1], wl)
                self.simulation.sim.setJointTargetVelocity(self.joints[2], wr)
                self.simulation.sim.setJointTargetVelocity(self.joints[3], wr)
                self.wait()


    def goto2(self, base, puntos):
        k1=0.15
        k2=0.0
        h=0.1
        for punto in puntos:
            xdes=punto[0]
            ydes=punto[1]
            vdes=1
            while (np.linalg.norm(punto - base.get_transform().pos())) > 0.5:
                print('error',(np.linalg.norm(punto - base.get_transform().pos())))

                T_base = base.get_transform()
                om = T_base.euler()[0].abg[2]
                pos = T_base.pos()

                [wl, nada, wr, nada2] = self.get_joint_speeds()
                Va, Wa = self.calcVW(wr=wr, wl=wl)

                ux=k2*(vdes-Va)-k1*(pos[0]-xdes)
                uy =k2*(vdes-Va) -k1* (pos[1] - ydes)
                V=ux*np.cos(om)+uy*np.sin(om)
                w=-(ux/h)*np.sin(om)+(uy/h)*np.cos(om)
                print(V,'V',w,'w')
                self.move(V,w)
                self.wait()


    def goto3(self, base, puntos):
        k1=0.15
        k2=0.0
        h=0.1
        for punto in puntos:
            xdes=punto[0]
            ydes=punto[1]
            vdes=1
            while (np.linalg.norm(punto - base.get_transform().pos())) > 0.5:
                print('error',(np.linalg.norm(punto - base.get_transform().pos())))

                T_base = base.get_transform()
                om = T_base.euler()[0].abg[2]
                pos = T_base.pos()

                [wl, nada, wr, nada2] = self.get_joint_speeds()
                Va, Wa = self.calcVW(wr=wr, wl=wl)
                jacob=np.array([(np.cos(om),-h*np.sin(om)),(np.sin(om),h*np.cos(om))])

                ux=k2*(vdes-Va)-k1*(pos[0]-xdes)
                uy =k2*(vdes-Va) -k1* (pos[1] - ydes)
                u=np.array([(ux),(uy)])
                [V,w]=np.linalg.inv(jacob)@u

                print('V',V,'w',w)
                self.move(V,w)
                self.wait()

    def move(self, v, w):
        r = self.wheel_radius
        b = self.width
        wl = (v - w * (b / 2)) / r
        wr = (v + w * (b / 2)) / r

        self.simulation.sim.setJointTargetVelocity(self.joints[0], wl)
        self.simulation.sim.setJointTargetVelocity(self.joints[1], wl)
        self.simulation.sim.setJointTargetVelocity(self.joints[2], wr)
        self.simulation.sim.setJointTargetVelocity(self.joints[3], wr)

    def forward(self):
        wheel_speeds = [3, 3, 3, 3]
        print('va')
        for i in range(4):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def backward(self):
        wheel_speeds = [-3, -3, -3, -3]
        for i in range(4):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def left(self):
        wheel_speeds = [0, 0, 3, 3]
        for i in range(4):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def right(self):
        wheel_speeds = [3, 3, 0, 0]
        for i in range(4):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def get_wheel_torques(self):
        torques = []
        # Caution: someone used the wrong direction for the joints
        # torques are considered positive when the robot exerts a force that makes
        # the robot go forward
        for i in range(4):
            tau = -self.simulation.sim.getJointForce(self.joints[i])
            torques.append(tau)
        return np.array(torques)

    def get_mean_wheel_torques(self):
        tau = 0.0
        # Caution: someone used the wrong direction for the joints
        for i in range(4):
            tau += -self.simulation.sim.getJointForce(self.joints[i])
        return tau/4.0

