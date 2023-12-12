#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
from robots.simulation import Simulation
from robots.camera import Camera


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()

    camera = Camera(simulation=simulation)
    camera.start(name='/sphericalVisionRGB/sensor')
    camera.save_image('test.png')



