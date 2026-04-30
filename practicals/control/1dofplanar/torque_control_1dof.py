"""
Simple torque based control 1
Use moveAbsJ moveQuintic

"""
import numpy as np
from robots.onedofrobot import OneDOFRobot
from robots.simulation import Simulation


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = OneDOFRobot(simulation=simulation)
    robot.start()
    total_time = 1.0
    # try to move robot with moveAbsJ (trapezoidal)
    q = [np.pi/2]
    robot.moveQuintic(q_target=q, total_time=total_time, precision=True, plot=True)
    # robot.moveAbsJ(q_target=q, precision=True, plot=True)
    # or move to a different place with a quintic polynomial and a time
    q = [np.pi / 4]
    robot.moveQuintic(q_target=q, total_time=total_time, precision=True, plot=True)
    # robot.moveAbsJ(q_target=q, precision=True, plot=True)
    simulation.stop()