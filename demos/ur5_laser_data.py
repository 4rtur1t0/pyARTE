#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5_velodyne.ttt scene before running this script.

The script provides an example to move a UR5 robot and get laser data.
 Laser data may be used to rectonstruct the environment, recognise objects... etc.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from sceneconfig.scene_configs_ur5 import init_simulation_UR5


if __name__ == "__main__":
    robot = init_simulation_UR5()
    q0 = np.pi / 8 * np.array([-6, 1, 3, 1, 2, 1])
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    laserdata = robot.get_laser_data()

    robot.stop_arm()

