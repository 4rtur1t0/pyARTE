#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

The script provides an example to move a UR5 robot along the null space.

The active space can be defined by selecting the rows of the Manipulator Jacobian.
The column in the V matrix can be selected, in this sense, one can move the robot along the different
vectors that form the basis of the null space.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from sceneconfig.scene_configs import init_simulation_UR5

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0



if __name__ == "__main__":
    robot = init_simulation_UR5()
    q0 = np.pi / 8 * np.array([-6, 1, 3, 1, 2, 1])
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    laserdata = robot.get_laser_data()

    # robot.plot_trajectories()
    robot.stop_arm()

