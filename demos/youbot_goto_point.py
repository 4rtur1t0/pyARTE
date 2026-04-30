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
from robots.objects import Dummy


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

    dummy = Dummy(clientID=clientID)
    dummy.start()



    # x, y, theta
    punto_destino = np.array([2, 1, 0.8])

    while True:
        pos_real = dummy.get_position()
        orient = dummy.get_orientation()
        state_real = np.array([pos_real[0], pos_real[1], orient[2]])
        print('Estado real: ', state_real)

        e = punto_destino-state_real
        if np.linalg.norm(e) < 0.1:
            break

        vw = 10*e
        print(vw)
        print(np.linalg.norm(vw))
        # move the robot base in speed
        robot.set_base_speed(vw[0], -vw[1], -vw[2])
        simulation.wait(10)





    # color = camera.get_color_name()
    # image = camera.get_image()
    #
    # print('Color is: ', color)
    # camera.save_image('image.png')

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    youbot_move()
