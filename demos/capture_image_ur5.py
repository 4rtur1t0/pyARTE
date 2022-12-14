#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
from sceneconfig.scene_configs_ur5 import init_simulation_UR5


def capture_image():
    robot = init_simulation_UR5()
    [image, resolution] = robot.get_image()
    robot.save_image(image=image, resolution=resolution, filename='test.png')
    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    capture_image()

