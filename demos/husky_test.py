#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez, Arturo Gil
@Time: February 2024
"""
from robots.accelerometer import Accelerometer
from robots.objects import CoppeliaObject
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
    # A dummy object ath the robot center
    robot_center = CoppeliaObject(simulation=simulation)
    robot_center.start(name='/HuskyCenter')
    # an accelerometer
    accel = Accelerometer(simulation=simulation)
    accel.start(name='/Accelerometer')

    # MOVEMENTSDA
    print('MOVING ROBOT')
    robot.move(v=-1, w=0.0)
    # Must wait till the speed is reached
    # approx 2 seconds (torques
    simulation.wait()

    #  TORQUES
    # now, obtain the mean torques or torques for each wheel
    # during 50 simulat steps.
    for i in range(50):
        tau = robot.get_mean_wheel_torques()
        axyz = accel.get_accel_data()
        T = robot_center.get_transform()
        print(T)
        print(tau)
        print(axyz)
        robot.wait()
    # robot.forward()
    # robot.wait(130)
    # robot.left()
    # robot.wait(130)
    # robot.right()
    # robot.wait(100)
    # # get lidar data
    for i in range(10):
        data = lidar.get_laser_data()
        lidar.save_pointcloud('lidar/simulated_pointcloud.pcd')
        lidar.down_sample()
        lidar.estimate_normals()
        lidar.draw_pointcloud()

    simulation.stop()


if __name__ == "__main__":
    simulate()