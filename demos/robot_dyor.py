#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
from sceneconfig.scene_configs_misc import init_simulation_mobile_robot


def move_robot():
    robot = init_simulation_mobile_robot()

    robot.forward()
    robot.wait(100)
    robot.backwards()
    robot.wait(100)
    robot.left()
    robot.wait(100)
    robot.right()
    robot.wait(100)

    robot.stop_simulation()


if __name__ == "__main__":
    move_robot()

