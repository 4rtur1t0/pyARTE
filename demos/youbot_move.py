#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/youbot.ttt scene before running this script.

@Authors: Arturo Gil arturo.gil@umh.es
@Time: April 2021
"""
import numpy as np
from robots.simulation import Simulation
from robots.youbot import YouBotBase, YouBotArm
from robots.grippers import YouBotGripper
from robots.camera import Camera


def youbot_move():
    """
    Move the robot base and arm joints
    """
    simulation = Simulation()
    simulation.start()
    robotbase = YouBotBase(simulation=simulation)
    robotbase.start()

    robotarm = YouBotArm(simulation=simulation)
    robotarm.start()

    gripper = YouBotGripper(simulation=simulation)
    gripper.start()
    camera = Camera(simulation=simulation)
    camera.start(name='/youBot/camera')

    # move the robot base in speed
    robotbase.set_base_speed(1, 0, 0)
    simulation.wait(100)
    robotbase.set_base_speed(-1, 0, 0)
    simulation.wait(100)
    robotbase.set_base_speed(0, 1, 0)
    simulation.wait(100)
    robotbase.set_base_speed(0, -1, 0)
    simulation.wait(100)
    robotbase.set_base_speed(0, 0, 1)
    simulation.wait(100)
    robotbase.set_base_speed(0, 0, -1)
    simulation.wait(100)
    # stop base
    robotbase.set_base_speed(0, 0, 0)
    simulation.wait(1)

    # now move the arm
    q0 = np.pi/16*np.array([16, 3, 6, 1, 1])
    robotarm.moveAbsJ(q0, precision=True)

    gripper.open(precision=True)
    gripper.close(precision=True)
    # simulation.wait(100)

    color = camera.get_color_name()
    image = camera.get_image()

    print('Color is: ', color)
    camera.save_image('image.png')

    simulation.stop()
    robotarm.plot_trajectories()


if __name__ == "__main__":
    youbot_move()
