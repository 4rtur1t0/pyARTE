#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/mobile_robot.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
from robots.robot_dyor import RobotDyor
from robots.simulation import Simulation


def move_robot():
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotDyor(clientID=clientID)
    robot.start()

    robot.forward()
    robot.wait(100)
    robot.backwards()
    robot.wait(100)
    robot.left()
    robot.wait(100)
    robot.right()
    robot.wait(100)

    simulation.stop()


if __name__ == "__main__":
    move_robot()

