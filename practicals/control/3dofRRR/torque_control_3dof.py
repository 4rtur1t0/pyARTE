"""
Simple torque based control 1

1 Plan a path (q1 to q2)
         position,
         speed
         acceleration.
2 At each time step, compute the inverse dynamics.

3 Apply torque along with a PID correction based on the error in position, speed and acceleration.

Open the 3dofrobot.ttt scene.
This is a normal 3 DOF antropomorphical robot.
"""
import numpy as np
from robots.sphericalrobotRRR import SphericalRobotRRR
from robots.simulation import Simulation
import matplotlib.pyplot as plt


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = SphericalRobotRRR(simulation=simulation)
    robot.start()
    total_time = 0.5
    q1 = np.array([np.pi/4, np.pi/4, np.pi/4])
    q2 = np.array([0.0, 0.0, 0.0])
    q3 = np.array([-np.pi / 4, np.pi / 2, -np.pi / 2])
    q4 = np.array([np.pi / 2, np.pi / 2, 0])
    q5 = np.array([0.0, 0.0, 0.0])
    plot = False
    precision = False
    robot.moveQuintic(q_target=q1, total_time=total_time, plot=plot)
    robot.moveQuintic(q_target=q2, total_time=total_time, plot=plot)
    robot.moveQuintic(q_target=q3, total_time=total_time, plot=plot)
    robot.moveQuintic(q_target=q4, total_time=total_time, plot=plot)
    robot.moveQuintic(q_target=q5, total_time=total_time, plot=plot)

    # also use moveAbsJ to perform movements
    robot.moveAbsJ(q1, plot=plot)
    robot.moveAbsJ(q2, plot=plot)
    robot.moveAbsJ(q3, plot=plot)
    robot.moveAbsJ(q4, plot=plot)
    robot.moveAbsJ(q5, plot=plot)

    robot.plot_states()
    simulation.stop()