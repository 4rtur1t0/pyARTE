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


def youbot_move():
    """
    Move the robot base and arm joints
    """
    simulation = Simulation()
    clientID = simulation.start()
    robot = YouBotRobot(clientID=clientID)
    robot.start()
    gripper = YouBotGripper(clientID=clientID)
    gripper.start()
    camera = Camera(clientID=clientID)
    camera.start()

    # move the robot base in speed
    robot.set_base_speed(1, 0, 0)
    simulation.wait(100)
    robot.set_base_speed(-1, 0, 0)
    simulation.wait(100)
    robot.set_base_speed(0, 1, 0)
    simulation.wait(100)
    robot.set_base_speed(0, -1, 0)
    simulation.wait(100)
    robot.set_base_speed(0, 0, 1)
    simulation.wait(100)
    robot.set_base_speed(0, 0, -1)
    simulation.wait(100)
    # stop base
    robot.set_base_speed(0, 0, 0)
    simulation.wait(1)

    # now move the arm
    q0 = np.pi/8*np.array([1, 1, 1, 1, 1])
    robot.set_joint_target_positions(q0, precision=True)
    simulation.wait(100)
    gripper.open(precision=True)
    simulation.wait(100)
    gripper.close(precision=True)
    simulation.wait(100)

    color = camera.get_color_name()
    print('Color is: ', color)
    camera.save_image('image.png')

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    youbot_move()
