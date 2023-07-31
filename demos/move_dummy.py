"""
Dummy movement of a Reference system using Euler angles XYZ in local coordinates
Use with scenes/euler.ttt
"""
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def move_reference_frame():
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    ref_frame = ReferenceFrame(simulation=simulation)
    ref_frame.start()
    position = [0.5, 0, 0.5]

    ref_frame.set_position(position)

    alpha = 0
    beta = 0
    gamma = 0

    for i in range(100):
        alpha += .005
        ref_frame.set_orientation([alpha, beta, gamma])

    for i in range(100):
        beta += .005
        ref_frame.set_orientation([alpha, beta, gamma])

    for i in range(100):
        gamma += .005
        ref_frame.set_orientation([alpha, beta, gamma])
    simulation.stop()


if __name__ == "__main__":
    move_reference_frame()