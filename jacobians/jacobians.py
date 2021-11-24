#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/youbot.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np


def eval_symbolic_jacobian_UR10(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    # q6 = q[6]
    Jw = np.array([[ 0, -np.sin(q1), -np.sin(q1), -np.sin(q1), np.sin(q1 + q2 + q3 + q4)/2 + np.sin(q2 - q1 + q3 + q4)/2, np.cos(q1)*np.cos(q2)*np.sin(q3)*np.sin(q4)*np.sin(q5) - np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q5) - np.cos(q5)*np.sin(q1) + np.cos(q1)*np.cos(q3)*np.sin(q2)*np.sin(q4)*np.sin(q5) + np.cos(q1)*np.cos(q4)*np.sin(q2)*np.sin(q3)*np.sin(q5)],
                 [ 0,  np.cos(q1),  np.cos(q1),  np.cos(q1), np.cos(q2 - q1 + q3 + q4)/2 - np.cos(q1 + q2 + q3 + q4)/2, np.cos(q1)*np.cos(q5) - np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q1)*np.sin(q5) + np.cos(q2)*np.sin(q1)*np.sin(q3)*np.sin(q4)*np.sin(q5) + np.cos(q3)*np.sin(q1)*np.sin(q2)*np.sin(q4)*np.sin(q5) + np.cos(q4)*np.sin(q1)*np.sin(q2)*np.sin(q3)*np.sin(q5)],
                 [ 1,        0,        0,        0, np.cos(q2 + q3 + q4), np.cos(q2 + q3 + q4 - q5)/2 - np.cos(q2 + q3 + q4 + q5)/2]])

    Jv = np.array([[(23 * np.sin(q1) * np.sin(q2) * np.sin(q3) * np.sin(q4)) / 200 - (23 * np.cos(q1) * np.cos(q5)) / 250 - (
            153 * np.sin(q1) * np.sin(q2)) / 250 - (143 * np.cos(q2) * np.sin(q1) * np.sin(q3)) / 250 - (
             143 * np.cos(q3) * np.sin(q1) * np.sin(q2)) / 250 - (23 * np.cos(q2) * np.cos(q3) * np.sin(q1) * np.sin(q4)) / 200 - (
             23 * np.cos(q2) * np.cos(q4) * np.sin(q1) * np.sin(q3)) / 200 - (23 * np.cos(q3) * np.cos(q4) * np.sin(q1) * np.sin(q2)) / 200 - (
             41 * np.cos(q1)) / 250 + (23 * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q1) * np.sin(q5)) / 250 - (
             23 * np.cos(q2) * np.sin(q1) * np.sin(q3) * np.sin(q4) * np.sin(q5)) / 250 - (
             23 * np.cos(q3) * np.sin(q1) * np.sin(q2) * np.sin(q4) * np.sin(q5)) / 250 - (
             23 * np.cos(q4) * np.sin(q1) * np.sin(q2) * np.sin(q3) * np.sin(q5)) / 250, (np.cos(q1) * (
            115 * np.cos(q2 + q3 + q4) - 46 * np.cos(q2 + q3 + q4 + q5) + 572 * np.cos(q2 + q3) + 612 * np.cos(q2) + 46 * np.cos(
        q2 + q3 + q4 - q5))) / 1000, np.cos(q1) * (
             (23 * np.cos(q2 + q3 + q4)) / 200 - (23 * np.cos(q2 + q3 + q4 + q5)) / 500 + (143 * np.cos(q2 + q3)) / 250 + (
                 23 * np.cos(q2 + q3 + q4 - q5)) / 500),
        np.cos(q1) * ((23 * np.cos(q2 + q3 + q4)) / 200 - (23 * np.cos(q2 + q3 + q4 + q5)) / 500 + (23 * np.cos(q2 + q3 + q4 - q5)) / 500),
            (23 * np.sin(q1) * np.sin(q5)) / 250 - (23 * np.cos(q1) * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.cos(q5)) / 250 + (
             23 * np.cos(q1) * np.cos(q2) * np.cos(q5) * np.sin(q3) * np.sin(q4)) / 250 + (
             23 * np.cos(q1) * np.cos(q3) * np.cos(q5) * np.sin(q2) * np.sin(q4)) / 250 + (
             23 * np.cos(q1) * np.cos(q4) * np.cos(q5) * np.sin(q2) * np.sin(q3)) / 250, 0],
            [(153 * np.cos(q1) * np.sin(q2)) / 250 - (41 * np.sin(q1)) / 250 - (23 * np.cos(q5) * np.sin(q1)) / 250 + (
            143 * np.cos(q1) * np.cos(q2) * np.sin(q3)) / 250 + (143 * np.cos(q1) * np.cos(q3) * np.sin(q2)) / 250 + (
             23 * np.cos(q1) * np.cos(q2) * np.cos(q3) * np.sin(q4)) / 200 + (23 * np.cos(q1) * np.cos(q2) * np.cos(q4) * np.sin(q3)) / 200 + (
             23 * np.cos(q1) * np.cos(q3) * np.cos(q4) * np.sin(q2)) / 200 - (23 * np.cos(q1) * np.sin(q2) * np.sin(q3) * np.sin(q4)) / 200 - (
             23 * np.cos(q1) * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q5)) / 250 + (
             23 * np.cos(q1) * np.cos(q2) * np.sin(q3) * np.sin(q4) * np.sin(q5)) / 250 + (
             23 * np.cos(q1) * np.cos(q3) * np.sin(q2) * np.sin(q4) * np.sin(q5)) / 250 + (
             23 * np.cos(q1) * np.cos(q4) * np.sin(q2) * np.sin(q3) * np.sin(q5)) / 250, (np.sin(q1) * (
            115 * np.cos(q2 + q3 + q4) - 46 * np.cos(q2 + q3 + q4 + q5) + 572 * np.cos(q2 + q3) + 612 * np.cos(q2) + 46 * np.cos(
        q2 + q3 + q4 - q5))) / 1000, np.sin(q1) * (
             (23 * np.cos(q2 + q3 + q4)) / 200 - (23 * np.cos(q2 + q3 + q4 + q5)) / 500 + (143 * np.cos(q2 + q3)) / 250 + (
                 23 * np.cos(q2 + q3 + q4 - q5)) / 500),
        np.sin(q1) * ((23 * np.cos(q2 + q3 + q4)) / 200 - (23 * np.cos(q2 + q3 + q4 + q5)) / 500 + (23 * np.cos(q2 + q3 + q4 - q5)) / 500),
        (23 * np.cos(q2) * np.cos(q5) * np.sin(q1) * np.sin(q3) * np.sin(q4)) / 250 - (
             23 * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.sin(q1)) / 250 - (23 * np.cos(q1) * np.sin(q5)) / 250 + (
             23 * np.cos(q3) * np.cos(q5) * np.sin(q1) * np.sin(q2) * np.sin(q4)) / 250 + (
             23 * np.cos(q4) * np.cos(q5) * np.sin(q1) * np.sin(q2) * np.sin(q3)) / 250, 0],
            [0, (23 * np.sin(q2 + q3 + q4 + q5)) / 500 - (23 * np.sin(q2 + q3 + q4)) / 200 - (143 * np.sin(q2 + q3)) / 250 - (
            153 * np.sin(q2)) / 250 - (23 * np.sin(q2 + q3 + q4 - q5)) / 500,
         (23 * np.sin(q2 + q3 + q4 + q5)) / 500 - (23 * np.sin(q2 + q3 + q4)) / 200 - (143 * np.sin(q2 + q3)) / 250 - (
             23 * np.sin(q2 + q3 + q4 - q5)) / 500,
        (23 * np.sin(q2 + q3 + q4 + q5)) / 500 - (23 * np.sin(q2 + q3 + q4)) / 200 - (23 * np.sin(q2 + q3 + q4 - q5)) / 500,
        (23 * np.sin(q2 + q3 + q4 + q5)) / 500 + (23 * np.sin(q2 + q3 + q4 - q5)) / 500, 0]])

    J = np.vstack((Jv, Jw))
    return J



def eval_symbolic_jacobian_lbr_kuka(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]
    
    Jw = np.array([[0, -np.sin(q1), np.cos(q1) * np.sin(q2), np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3),
                    np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2),
                    np.sin(q5) * (np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) - np.cos(q5) * (
             np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)), np.cos(q6) * (np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2)) - np.sin(q6) * (
             np.cos(q5) * (np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) + np.sin(q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)))],
            [0, np.cos(q1), np.sin(q1) * np.sin(q2), np.cos(q2) * np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q3), np.cos(q4) * np.sin(q1) * np.sin(q2) - np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)),
            np.cos(q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3)) - np.sin(q5) * (np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)), np.sin(q6) * (
             np.cos(q5) * (np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) + np.sin(q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3))) - np.cos(q6) * (
             np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2))],
            [1, 0, np.cos(q2), -np.sin(q2) * np.sin(q3), np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4), np.cos(q5) * np.sin(q2) * np.sin(q3) - np.sin(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)),
            np.sin(q6) * (np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5)) + np.cos(q6) * (np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))]])

    Jv = np.array([[(111 * np.cos(q6) * (np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2))) / 1000 - (
             21 * np.sin(q1) * np.sin(q2)) / 50 - (111 * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) + np.sin(
        q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3)))) / 1000 + (
             2 * np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1))) / 5 - (
             2 * np.cos(q4) * np.sin(q1) * np.sin(q2)) / 5, np.cos(q1) * ((21 * np.cos(q2)) / 50 + (2 * np.cos(q2) * np.cos(q4)) / 5 + (
            111 * np.sin(q6) * (np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (111 * np.cos(q6) * (
            np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))) / 1000 + (2 * np.cos(q3) * np.sin(q2) * np.sin(q4)) / 5),
            (2 * np.cos(q3) * np.sin(q1) * np.sin(q4)) / 5 + (2 * np.cos(q1) * np.cos(q2) * np.sin(q3) * np.sin(q4)) / 5 + (
             111 * np.cos(q3) * np.cos(q6) * np.sin(q1) * np.sin(q4)) / 1000 + (
             111 * np.sin(q1) * np.sin(q3) * np.sin(q5) * np.sin(q6)) / 1000 + (
             111 * np.cos(q1) * np.cos(q2) * np.cos(q6) * np.sin(q3) * np.sin(q4)) / 1000 - (
             111 * np.cos(q1) * np.cos(q2) * np.cos(q3) * np.sin(q5) * np.sin(q6)) / 1000 - (
             111 * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.sin(q1) * np.sin(q6)) / 1000 - (
             111 * np.cos(q1) * np.cos(q2) * np.cos(q4) * np.cos(q5) * np.sin(q3) * np.sin(q6)) / 1000,
            - (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3)) * ((2 * np.cos(q2) * np.cos(q4)) / 5 + (111 * np.sin(q6) * (
             np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (
            111 * np.cos(q6) * (np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))) / 1000 + (2 * np.cos(q3) * np.sin(q2) * np.sin(q4)) / 5) - np.sin(
     q2) * np.sin(q3) * ((111 * np.cos(q6) * (np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2))) / 1000 - (
        111 * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(
                                  q2) * np.sin(q4)) + np.sin(q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(
                              q3)))) / 1000 + (2 * np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1))) / 5 - (
                                  2 * np.cos(q4) * np.sin(q1) * np.sin(q2)) / 5), -(111 * np.sin(q6) * (
            np.cos(q3) * np.cos(q5) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.cos(q5) * np.sin(q3) + np.cos(q1) * np.sin(q2) * np.sin(q4) * np.sin(
        q5) - np.cos(q4) * np.sin(q1) * np.sin(q3) * np.sin(q5) + np.cos(q1) * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q5))) / 1000,
 - (np.sin(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) - np.cos(q5) * np.sin(q2) * np.sin(q3)) * ((111 * np.cos(q6) * (
             np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2))) / 1000 - (
    111 * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) + np.sin(q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3)))) / 1000) - (np.sin(q5) * (
                 np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) - np.cos(
         q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3))) * ((111 * np.sin(q6) * (
             np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (
                                                                                 111 * np.cos(q6) * (
                                                                                     np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(
                                                                                 q2) * np.sin(q4))) / 1000), 0],
        [(21 * np.cos(q1) * np.sin(q2)) / 50 + (111 * np.cos(q6) * (
            np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2))) / 1000 - (
             111 * np.sin(q6) * (np.cos(q5) * (
                 np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) + np.sin(
         q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)))) / 1000 + (
             2 * np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3))) / 5 + (
             2 * np.cos(q1) * np.cos(q4) * np.sin(q2)) / 5, np.sin(q1) * ((21 * np.cos(q2)) / 50 + (2 * np.cos(q2) * np.cos(q4)) / 5 + (
            111 * np.sin(q6) * (
                np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (
                                                                          111 * np.cos(q6) * (
                                                                              np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(
                                                                          q2) * np.sin(q4))) / 1000 + (
                                                                          2 * np.cos(q3) * np.sin(q2) * np.sin(q4)) / 5),
 (2 * np.cos(q2) * np.sin(q1) * np.sin(q3) * np.sin(q4)) / 5 - (111 * np.cos(q1) * np.cos(q3) * np.cos(q6) * np.sin(q4)) / 1000 - (
             2 * np.cos(q1) * np.cos(q3) * np.sin(q4)) / 5 - (111 * np.cos(q1) * np.sin(q3) * np.sin(q5) * np.sin(q6)) / 1000 + (
             111 * np.cos(q1) * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.sin(q6)) / 1000 + (
             111 * np.cos(q2) * np.cos(q6) * np.sin(q1) * np.sin(q3) * np.sin(q4)) / 1000 - (
             111 * np.cos(q2) * np.cos(q3) * np.sin(q1) * np.sin(q5) * np.sin(q6)) / 1000 - (
             111 * np.cos(q2) * np.cos(q4) * np.cos(q5) * np.sin(q1) * np.sin(q3) * np.sin(q6)) / 1000,
 - (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)) * ((2 * np.cos(q2) * np.cos(q4)) / 5 + (111 * np.sin(q6) * (
             np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (
                                                                    111 * np.cos(q6) * (
                                                                        np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(
                                                                    q4))) / 1000 + (
                                                                    2 * np.cos(q3) * np.sin(q2) * np.sin(q4)) / 5) - np.sin(
     q2) * np.sin(q3) * ((111 * np.cos(q6) * (
             np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2))) / 1000 - (
                                  111 * np.sin(q6) * (np.cos(q5) * (
                                      np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(
                                  q2) * np.sin(q4)) + np.sin(q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(
                              q3)))) / 1000 + (2 * np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3))) / 5 + (
                                  2 * np.cos(q1) * np.cos(q4) * np.sin(q2)) / 5), -(111 * np.sin(q6) * (
            np.cos(q2) * np.cos(q5) * np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q3) * np.cos(q5) + np.cos(q1) * np.cos(q4) * np.sin(q3) * np.sin(
        q5) + np.sin(q1) * np.sin(q2) * np.sin(q4) * np.sin(q5) + np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q1) * np.sin(q5))) / 1000,
 - (np.sin(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) - np.cos(q5) * np.sin(q2) * np.sin(q3)) * ((111 * np.cos(q6) * (
             np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2))) / 1000 - (
                                                                                                              111 * np.sin(
                                                                                                          q6) * (
                                                                                                                          np.cos(q5) * (
                                                                                                                              np.cos(q4) * (
                                                                                                                                  np.sin(q1) * np.sin(
                                                                                                                              q3) - np.cos(
                                                                                                                              q1) * np.cos(
                                                                                                                              q2) * np.cos(
                                                                                                                              q3)) - np.cos(
                                                                                                                          q1) * np.sin(
                                                                                                                          q2) * np.sin(
                                                                                                                          q4)) + np.sin(
                                                                                                                      q5) * (
                                                                                                                                      np.cos(q3) * np.sin(
                                                                                                                                  q1) + np.cos(
                                                                                                                                  q1) * np.cos(
                                                                                                                                  q2) * np.sin(
                                                                                                                                  q3)))) / 1000) - (
             np.sin(q5) * (
                 np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) - np.cos(
         q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3))) * ((111 * np.sin(q6) * (
             np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (
                                                                                 111 * np.cos(q6) * (
                                                                                     np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(
                                                                                 q2) * np.sin(q4))) / 1000), 0],
        [0, (2 * np.cos(q2) * np.cos(q3) * np.sin(q4)) / 5 - (2 * np.cos(q4) * np.sin(q2)) / 5 - (21 * np.sin(q2)) / 50 - (
            111 * np.cos(q4) * np.cos(q6) * np.sin(q2)) / 1000 + (111 * np.cos(q2) * np.cos(q3) * np.cos(q6) * np.sin(q4)) / 1000 + (
             111 * np.cos(q2) * np.sin(q3) * np.sin(q5) * np.sin(q6)) / 1000 - (
             111 * np.cos(q5) * np.sin(q2) * np.sin(q4) * np.sin(q6)) / 1000 - (
             111 * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.sin(q6)) / 1000, -(np.sin(q2) * (
            400 * np.sin(q3) * np.sin(q4) + 111 * np.cos(q6) * np.sin(q3) * np.sin(q4) - 111 * np.cos(q3) * np.sin(q5) * np.sin(q6) - 111 * np.cos(
        q4) * np.cos(q5) * np.sin(q3) * np.sin(q6))) / 1000,
 (2 * np.cos(q3) * np.cos(q4) * np.sin(q2)) / 5 - (2 * np.cos(q2) * np.sin(q4)) / 5 - (111 * np.cos(q2) * np.cos(q6) * np.sin(q4)) / 1000 + (
             111 * np.cos(q3) * np.cos(q4) * np.cos(q6) * np.sin(q2)) / 1000 + (
             111 * np.cos(q2) * np.cos(q4) * np.cos(q5) * np.sin(q6)) / 1000 + (
             111 * np.cos(q3) * np.cos(q5) * np.sin(q2) * np.sin(q4) * np.sin(q6)) / 1000, (111 * np.sin(q6) * (
            np.cos(q5) * np.sin(q2) * np.sin(q3) - np.cos(q2) * np.sin(q4) * np.sin(q5) + np.cos(q3) * np.cos(q4) * np.sin(q2) * np.sin(q5))) / 1000,
 (111 * np.cos(q2) * np.cos(q5) * np.cos(q6) * np.sin(q4)) / 1000 - (111 * np.cos(q2) * np.cos(q4) * np.sin(q6)) / 1000 - (
             111 * np.cos(q3) * np.sin(q2) * np.sin(q4) * np.sin(q6)) / 1000 + (
             111 * np.cos(q6) * np.sin(q2) * np.sin(q3) * np.sin(q5)) / 1000 - (
             111 * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.cos(q6) * np.sin(q2)) / 1000, 0]])

    J = np.vstack((Jv, Jw))
    return J


