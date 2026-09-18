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
from robots.camera import Camera

def observe_aruco(camera):
    id, Tca = camera.detect_closer_aruco(show=False, aruco_size=0.1)

    return id, Tca

def simulate():
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # A dummy object ath the robot center
    robot_center = CoppeliaObject(simulation=simulation)
    robot_center.start(name='/HuskyCenter')
    # an accelerometer
    accel = Accelerometer(simulation=simulation)
    accel.start(name='/Accelerometer')

    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=60)
    camera.start(name='/camera')


    simulation.wait(steps=10)

    #  TORQUES
    # now, obtain the mean torques or torques for each wheel
    # during 50 simulat steps.
    for i in range(350):
        # tau = robot.get_mean_wheel_torques()
        # axyz = accel.get_accel_data()
        T = robot_center.get_transform()
        id_aruco, Tca = observe_aruco(camera=camera)
        robot.move(v=0.1, w=0.7)
        # print(T)
        # print(tau)
        # print(axyz)
        robot.wait()

    simulation.stop()


if __name__ == "__main__":
    simulate()