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
from robots.simulation import Simulation
from robots.ur5 import RobotUR5
from robots.velodyne import Velodyne

if __name__ == "__main__":
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotUR5(clientID=clientID)
    robot.start()
    lidar = Velodyne(clientID=clientID)
    lidar.start()

    q0 = np.pi / 16 * np.array([-6, 1, 3, 1, 2, 1])
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    laserdata = lidar.get_laser_data()
    print('Received Laser Data')
    print(laserdata)
    simulation.stop()

