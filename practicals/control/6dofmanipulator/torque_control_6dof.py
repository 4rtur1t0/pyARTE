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
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.simulation import Simulation
import matplotlib.pyplot as plt



if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    total_time = 0.5
    q1 = np.array([0.0, 0.0, 0.0, 0.0, np.pi/2, 0.0])
    # q1 = np.array([np.pi/2, -np.pi/4, np.pi/8, -np.pi/8, np.pi/12, np.pi/20])
    q2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q3 = np.array([-np.pi/4, np.pi/8, -np.pi/2, np.pi/8, np.pi/8, np.pi/8])
    q4 = np.array([np.pi/4, -np.pi/8, 0, -np.pi/8, -np.pi/8, -np.pi/8])
    q5 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    plot = False
    precision = True
    # select a larger velocity that the standard robot speed limits
    speed_factor = 1.0
    # Use moveQuintic to move the robot using a quintic planner
    # Only the total_time of the movement is specified
    # The max speeds or accels are unlimited
    # robot.moveQuintic(q_target=q1, total_time=total_time, precision=precision, plot=plot)
    # robot.moveQuintic(q_target=q2, total_time=total_time, precision=precision, plot=plot)
    # robot.moveQuintic(q_target=q3, total_time=total_time, precision=precision, plot=plot)
    # robot.moveQuintic(q_target=q4, total_time=total_time, precision=precision, plot=plot)
    # robot.moveQuintic(q_target=q5, total_time=total_time, precision=precision, plot=plot)

    # also use moveAbsJ to perform movements
    # To options exist for moveAbsJ: a) select true_trapezoidal=True for
    # a real true trapezoidal --> the trajectory is more difficult to follow by the robot
    # an approx. trapezoidal --> once a t_coord time is found, the trajectories are
    # computed with a quintic planner. The max speeds and accelerations are approximately considered
    robot.moveAbsJ(q1, precision=precision, plot=plot, true_trapezoidal=False, speed_factor=speed_factor)
    robot.moveAbsJ(q2, precision=precision, plot=plot, true_trapezoidal=False, speed_factor=speed_factor)
    robot.moveAbsJ(q3, precision=precision, plot=plot, true_trapezoidal=False, speed_factor=speed_factor)
    robot.moveAbsJ(q4, precision=precision, plot=plot, true_trapezoidal=False, speed_factor=speed_factor)
    robot.moveAbsJ(q5, precision=precision, plot=plot, true_trapezoidal=False, speed_factor=speed_factor)
    simulation.stop()