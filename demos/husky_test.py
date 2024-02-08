#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez, Arturo Gil
@Time: February 2024
"""
from robots.ouster import Ouster
from robots.simulation import Simulation
from robots.husky import HuskyRobot


def simulate():
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # Simulate a LiDAR
    lidar = Ouster(simulation=simulation)
    lidar.start(name='/OS1')

    robot.move(v=1, w=0)
    simulation.wait_time(30)
    # robot.forward()
    # robot.wait(130)
    # robot.left()
    # robot.wait(130)
    # robot.right()
    # robot.wait(100)
    #
    # # get lidar data
    # data = lidar.get_laser_data()
    # print('Received Laser Data')
    # try:
    #     print(data.shape)
    #     print(data.dtype)
    # except:
    #     print('Unknown data type')

    simulation.stop()


if __name__ == "__main__":
    simulate()