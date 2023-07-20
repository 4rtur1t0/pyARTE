#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil arturo.gil@umh.es
@Time: April 2021
"""
import numpy as np
from robots.simulation import Simulation
from robots.youbot import YouBotRobot
from robots.grippers import YouBotGripper
from robots.camera import Camera
from robots.laserscans import LaserScanner2D


def youbot_move():
    """
    Move the robot base and arm joints
    """
    simulation = Simulation()
    clientID = simulation.start()
    robot = YouBotRobot(clientID=clientID)
    robot.start()
    # gripper = YouBotGripper(clientID=clientID)
    # gripper.start()
    # camera = Camera(clientID=clientID)
    # camera.start()

    pos0, ori0 = robot.get_true_position_and_orientation()
    robot.set_base_speed(0.5, 0, 0.1)
    simulation.wait(250)
    pos1, ori1 = robot.get_true_position_and_orientation()




    laser = LaserScanner2D(clientID=clientID)
    laser.start()
    distancies = laser.get_laser_data()

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    youbot_move()
