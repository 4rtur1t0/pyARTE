#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/irb140_two_robots.ttt scene before running this script.

Two abb robots and a pick and place application with palletizing.

In this version, python Threads are used so that both robots can be used independently.

CAUTION: One of the two proximity sensors shows an erratic beheviour at the beginning of the simulation.

@Authors: Arturo Gil
@Time: April 2022
"""
import threading
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad, GripperRG2
from robots.proxsensor import ProxSensor
from robots.simulation import Simulation

global clientID


def pick1(robot, gripper):
    tp1 = Vector([0.43, -0.28, 0.45])  # approximation
    tp2 = Vector([0.43, -0.28, 0.35])  # pick
    to = Euler([0, np.pi, 0])
    q0 = np.array([0, 0, 0, 0, 0, 0])
    robot.moveAbsJ(q_target=q0)
    gripper.open(precision=True)
    robot.moveJ(target_position=tp1, target_orientation=to)
    robot.moveL(target_position=tp2, target_orientation=to, precision='last')
    gripper.close(precision=True)
    robot.moveL(target_position=tp1, target_orientation=to)


def place1(robot, gripper):
    tp1 = Vector([-0.1, -0.65, 0.45])
    tp2 = Vector([-0.1, -0.65, 0.4])
    to = Euler([0, np.pi, 0])

    robot.moveJ(target_position=tp1, target_orientation=to)
    robot.moveL(target_position=tp2, target_orientation=to)
    gripper.open(precision=True)
    robot.moveL(target_position=tp1, target_orientation=to)


def pick2(robot, gripper):
    tp1 = Vector([0.5, 0.0, 0.45])  # approximation
    tp2 = Vector([0.5, 0.0, 0.23])  # pick
    to = Euler([0, np.pi, np.pi/2])

    gripper.open(precision=True)
    robot.moveJ(target_position=tp1, target_orientation=to)
    robot.moveL(target_position=tp2, target_orientation=to, precision='last')
    gripper.close(precision=True)
    robot.moveL(target_position=tp1, target_orientation=to)


def place2(robot, gripper):
    tp1 = Vector([-0.1, -0.6, 0.45])
    tp2 = Vector([-0.1, -0.6, 0.4])
    to = Euler([0, np.pi, 0])

    robot.moveJ(target_position=tp1, target_orientation=to)
    robot.moveL(target_position=tp2, target_orientation=to)
    gripper.open(precision=True)
    robot.moveL(target_position=tp1, target_orientation=to)


def pick_and_place1():

    global clientID
    robot1 = RobotABBIRB140(simulation=simulation)
    robot1.start(base_name='/IRB140')

    conveyor_sensor1 = ProxSensor(simulation=simulation)
    conveyor_sensor1.start(name='/conveyor/prox_sensor')

    gripper1 = GripperRG2(simulation=simulation)
    gripper1.start(name='/IRB140/RG2/RG2_openCloseJoint')

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot1.moveAbsJ(q_target=q0, precision=True)

    n_pieces = 27
    for i in range(n_pieces):
        while True:
            if conveyor_sensor1.is_activated():
                break
            simulation.wait()
        robot1.moveAbsJ(q_target=q0, precision=True)
        pick1(robot1, gripper1)
        robot1.moveAbsJ(q_target=q0, precision=True)
        place1(robot1, gripper1)


def pick_and_place2():
    global clientID

    robot2 = RobotABBIRB140(simulation=simulation)
    robot2.start(base_name='/IRB140_2')

    conveyor_sensor2 = ProxSensor(simulation=simulation)
    conveyor_sensor2.start(name='/conveyor_2/prox_sensor_2')

    gripper2 = SuctionPad(simulation=simulation)
    gripper2.start()

    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    robot2.moveAbsJ(q_target=q0, precision=True)

    n_pieces = 27
    for i in range(n_pieces):
        while True:
            if conveyor_sensor2.is_activated():
                break
            simulation.wait()
        robot2.moveAbsJ(q_target=q0, precision=True)
        pick2(robot2, gripper2)
        robot2.moveAbsJ(q_target=q0, precision=True)
        place2(robot2, gripper2)


if __name__ == "__main__":
    global clientID
    simulation = Simulation()
    simulation.start()

    r1 = threading.Thread(target=pick_and_place1, args=())
    r2 = threading.Thread(target=pick_and_place2, args=())

    r1.start()
    r2.start()

    # while True:
    #     if fin1 and fin2:
    #         print('DONE closing connection')
    #         # Stop arm and simulation
    #         robot1.stop_arm()
    #         robot1.stop_simulation()
    #         break

    # simulation.stop()
