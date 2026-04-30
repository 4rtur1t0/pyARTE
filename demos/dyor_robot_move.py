#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/dyor_robot.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
from robots.robot_dyor import RobotDyor
from robots.simulation import Simulation


def move_robot():
    simulation = Simulation()
    simulation.start()
    robot = RobotDyor(simulation=simulation)
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

