#!/usr/bin/env python
# encoding: utf-8
"""
Please open the irb140_project_2425.ttt scene before running this script.

The code beneath must be completed by the student in order to produce different assembly of the ARUCO Tetris pieces
according to their ids.

@Authors: Arturo Gil
@Time: October 2024
"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import SuctionPad
from robots.objects import ReferenceFrame
from robots.simulation import Simulation
from robots.camera import Camera


def detect_ARUCO_transform(robot, camera, show=True, aruco_size=0.03):
    """
    In case we want a list to all ARUCOS along with its transformation
    """
    id, Tca = camera.detect_closer_aruco(show=show, aruco_size=aruco_size)
    if id is None:
        print('could not find piece')
        return None, None
    q = robot.get_joint_positions()
    Te = robot.directkinematics(q)
    # transformacion ventosa--> cámara
    Tec = HomogeneousMatrix(Vector([0, -0.05, 0.0]), Euler([0, 0, -np.pi / 2]))
    # transformacion total
    T = Te * Tec * Tca
    return id, T


def perform_approx(robot, suction, T, operation, speed_factor):
    robot.moveJ(target_position=T.pos(), target_orientation=T.R(),
                speed_factor=speed_factor, precision=False)
    suction_action(suction, operation, 'approx')


def perform_target(robot, suction, T, operation, speed_factor):
    # the final pick operation
    robot.moveL(target_position=T.pos(), target_orientation=T.R(),
                speed_factor=speed_factor, precision=True, plot=False, debug=False)
    suction_action(suction, operation, 'target')


def perform_aruco(robot, camera, suction, operation, speed_factor):
    # if observe is true, the initial pick target position is refined by an ARUCO observation
    observe_aruco = operation['aruco'].get('observe_aruco')
    if observe_aruco:
        aruco_size = operation['aruco'].get('aruco_size')
        id, T = detect_ARUCO_transform(robot=robot, camera=camera,
                                       show=False,
                                       aruco_size=aruco_size)
        if id is None:
            print('NO ARUCO COULD BE DETECTED, EXITING')
            return False
        Trel = operation['aruco'].get('Trel')
        if Trel is None:
            print('No Tpick transform was definedbottle could be detected')
            return False
        T = T * Trel
        # the final pick operation
        robot.moveL(target_position=T.pos(), target_orientation=T.R(),
                    speed_factor=speed_factor, precision=True, plot=False, debug=False)
    suction_action(suction, operation, 'aruco')


def perform_exit(robot, suction, T, operation, speed_factor):
    # the final pick operation
    robot.moveL(target_position=T.pos(), target_orientation=T.R(),
                speed_factor=speed_factor, precision=True, plot=False, debug=False)
    suction_action(suction, operation, 'exit')


def suction_action(suction, operation, keyword):
    suction_action = operation[keyword].get('suction')
    if suction_action == 'off':
        suction.open()
    elif suction_action == 'on':
        suction.close()


def perform_operation(robot, camera, suction, operation, speed_factor):
    """
    Pick the object from target point from_target, and leave it in to_target
    if observe is True, the robot adjusts the target point starting at from_target
    according to the observation of the ARUCO
    """
    # find target, approx and exit
    target = operation['target']
    Ttarget = HomogeneousMatrix(target['tp'], target['to'])
    # approx
    approx = operation['approx']
    Tapprox = HomogeneousMatrix(approx['tp'], approx['to'])
    # exit
    exit = operation['exit']
    Texit = HomogeneousMatrix(exit['tp'], exit['to'])
    # compute all the points in absolute coordinates
    Tapprox = Ttarget * Tapprox
    Texit = Ttarget * Texit

    # perform operations
    perform_approx(robot, suction, Tapprox, operation, speed_factor)
    perform_target(robot, suction, Ttarget, operation, speed_factor)
    perform_aruco(robot, camera, suction, operation, speed_factor)
    perform_exit(robot, suction, Texit, operation, speed_factor)


def define_operations():
    # target: the main target point to reach (at position and orientation)
    # suction: the command on the suction cup (on or off)
    # if observe_aruco is True, once reached the target, an ARUCO is observed and the relative transformation is
    # commanded
    # approx and exit points are defined in coords relative to the target point

    # A, agafar la battery case i deixarla sobre el centrador
    battery_case_pick_A = {'target': {'tp': Vector([0.5, 0.15, 0.22]),
                                      'to': Euler([0, np.pi/2, 0]),
                                      'suction': 'on'},
                           'aruco': {'observe_aruco': True,
                                     'aruco_size': 0.02,
                                     'Trel': HomogeneousMatrix(Vector([0, 0, 0]),
                                                               Euler([0, np.pi, np.pi/2])),
                                     'suction': 'on'},
                           'approx': {'tp': Vector([0, 0, -0.05]),
                                      'to': Euler([0, 0, 0]),
                                      'suction': 'on'},
                           'exit': {'tp': Vector([-0.3, 0.0, 0.0]),
                                    'to': Euler([0, 0, 0]),
                                    'suction': 'on'}}
    battery_case_place_A = {'target': {'tp': Vector([0.15, 0.625, 0.26]),
                                       'to': Euler([0, np.pi/2, np.pi/6]),
                                       'suction': 'off'},
                            'aruco': {'observe_aruco': False},
                            'approx': {'tp': Vector([-0.1, 0, 0.0]),
                                       'to': Euler([0, 0, 0]),
                                       'suction': 'on'},
                            'exit': {'tp': Vector([-0.3, -0.1, -0.05]),
                                     'to': Euler([0, 0, 0]),
                                     'suction': 'off'}}

    battery_pick = {'target': {'tp': Vector([0.55, 0.35, 0.12]),
                               'to': Euler([0, np.pi/2, -np.pi/2]),
                               'suction': 'on'},
                    'aruco': {'observe_aruco': True,
                              'aruco_size': 0.01,
                              'Trel': HomogeneousMatrix(Vector([0, 0, 0]),
                                      Euler([0, np.pi, np.pi/2])),
                              'suction': 'on'},
                    'approx': {'tp': Vector([0, 0, -0.01]),
                               'to': Euler([0, 0, 0]),
                               'suction': 'on'},
                    'exit': {'tp': Vector([0.0, -0.3, -0.01]),
                             'to': Euler([0, 0, 0]),
                             'suction': 'on'}}

    battery_place = {'target': {'tp': Vector([0.17, 0.58, 0.35]),
                                'to': Euler([0, np.pi/2, 0]),
                                'suction': 'off'},
                     'aruco': {'observe_aruco': False},
                     'approx': {'tp': Vector([-0.05, 0, 0.0]),
                                'to': Euler([0, 0, 0]),
                                'suction': 'on'},
                     'exit': {'tp': Vector([-0.3, -0.1, -0.01]),
                              'to': Euler([0, 0, 0]),
                              'suction': 'off'}}

    # A, agafar la battery case i deixarla sobre la conveyor
    battery_case_pick_B = {'target': {'tp': Vector([0.05, 0.65, 0.26]),
                                       'to': Euler([0, np.pi/2, np.pi/6]),
                                       'suction': 'off'},
                           'aruco': {'observe_aruco': True,
                                     'aruco_size': 0.02,
                                     'Trel': HomogeneousMatrix(Vector([0, 0, 0]),
                                                               Euler([0, np.pi, np.pi / 2])),
                                     'suction': 'on'},
                           'approx': {'tp': Vector([0, 0, -0.05]),
                                      'to': Euler([0, 0, 0]),
                                      'suction': 'on'},
                           'exit': {'tp': Vector([-0.3, 0.0, 0.0]),
                                    'to': Euler([0, 0, 0]),
                                    'suction': 'on'}}
    battery_case_place_B = {'target': {'tp': Vector([0.4, -0.5, 0.26]),
                                       'to': Euler([np.pi / 2, 0, -np.pi / 2]),
                                       'suction': 'off'},
                            'aruco': {'observe_aruco': False},
                            'approx': {'tp': Vector([-0.1, 0, 0.0]),
                                       'to': Euler([0, 0, 0]),
                                       'suction': 'on'},
                            'exit': {'tp': Vector([-0.3, -0.1, -0.05]),
                                     'to': Euler([0, 0, 0]),
                                     'suction': 'off'}}
    return battery_case_pick_A, battery_case_place_A, battery_pick, battery_place, battery_case_pick_B, battery_case_place_B


def assembly_operation():
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()
    camera = Camera(simulation=simulation, resolution=1200, fov_degrees=45)
    camera.start(name='/IRB140/camera')
    suction = SuctionPad(simulation=simulation)
    suction.start()
    # TCP DE LA VENTOSA! OJO!: debe ser el adecuado para la escena
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0.0, 0.05]), Euler([0, 0, 0])))
    # Se mueve el robot a una posición articular q
    q = np.array([0, 0, 0, 0, 0, 0])
    robot.moveAbsJ(q_target=q, precision=False, speed_factor=3.0)

    battery_case_pick_A, battery_case_place_A, \
    battery_pick, battery_place, \
    battery_case_pick_B, battery_case_place_B = define_operations()

    n_batt_cases = 3
    n_batts_per_case = 3  # number of batts per case
    for i in range(n_batt_cases):
        perform_operation(robot=robot, camera=camera, suction=suction,
                          operation=battery_case_pick_A, speed_factor=3.0)
        perform_operation(robot=robot, camera=camera, suction=suction,
                          operation=battery_case_place_A, speed_factor=3.0)
        for j in range(n_batts_per_case):
            # pick batt and place inside case
            perform_operation(robot=robot, camera=camera, suction=suction,
                              operation=battery_pick, speed_factor=3.0)
            perform_operation(robot=robot, camera=camera, suction=suction,
                              operation=battery_place, speed_factor=3.0)
        # now pick the case and place it on the other conveyor belt
        perform_operation(robot=robot, camera=camera, suction=suction,
                          operation=battery_case_pick_B, speed_factor=0.5)
        perform_operation(robot=robot, camera=camera, suction=suction,
                          operation=battery_case_place_B, speed_factor=0.5)
    simulation.stop()


if __name__ == "__main__":
    assembly_operation()


