#!/usr/bin/env python
# encoding: utf-8
"""
A number of miscellaneous tools
@Authors: Arturo Gil
@Time: April 2021

"""
import matplotlib.pyplot as plt
import numpy as np


def plot(x, title='Untitled', block=False):
    plt.figure()
    x = np.array(x)
    plt.plot(x)
    plt.title(title)
    plt.show(block=block)