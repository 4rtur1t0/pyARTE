#!/usr/bin/env python
# encoding: utf-8
"""
A class to print errors
@Authors: Arturo Gil
@Time: April 2026
"""


class ErrorPrint:
    def __init__(self):
        return

    def print(self,  message, color='red'):
        if color == 'red':
            print("\033[91m" + message + "\033[0m")
        elif color == 'green':
            print("\033[92m" + message + "\033[0m")
        elif color == 'blue':
            print("\033[94m" + message + "\033[0m")
        else:
            print(message)