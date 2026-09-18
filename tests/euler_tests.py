#!/usr/bin/env python
# encoding: utf-8
"""
Test Euler angles
@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler


def test_euler():
    """
    Test Euler angles. Generate solutions and test that both solutions yield the same rotation matrix.
    One of the solutions in e2 should also match the initial solution in e1.
    """
    print('TESTING NORMAL CASES')
    np.random.seed(0)
    for i in range(1000):
        a = np.random.uniform(-np.pi, np.pi)
        b = np.random.uniform(-np.pi, np.pi)
        g = np.random.uniform(-np.pi, np.pi)
        # excluding the degenerate case beta=+-pi/2, so that we can check
        if np.abs(np.abs(np.sin(b))-1.0) < 0.0001:
            continue
        e1 = Euler([a, b, g])
        R1 = e1.R()
        e2 = R1.euler()
        Ra = e2[0].R()
        Rb = e2[1].R()
        ka = abs((Ra - R1).det())
        kb = abs((Rb - R1).det())
        if ka > 0.0001 and kb > 0.0001:
            print('TEST FAILED')
            return
    print('TEST PASSED')


def test_degenerate1():
    """
    Test some degenerate cases.
    """
    print('TESTING DEGENERATE CASE 1')
    e1 = Euler([1.5, np.pi/2, 0.5])
    R1 = e1.R()
    # actually, being a degenerate case, e1 and e2 are different
    # en aquest cas, només comprovem que l'orientació assolida en ambdós casos és la mateixa
    e2 = R1.euler()
    Ra = e2[0].R()
    Rb = e2[1].R()
    a = abs((Ra - R1).det())
    b = abs((Rb - R1).det())
    if  a < 0.0001 and b < 0.0001:
        print('TEST PASSED')
    else:
        print('TEST FAILED')

def test_degenerate2():
    """
    Test some degenerate cases.
    """
    print('TESTING DEGENERATE CASE 2')
    e1 = Euler([1.5, -np.pi/2, 0.5])
    R1 = e1.R()
    # actually, being a degenerate case, e1 and e2 are different
    # en aquest cas, només comprovem que l'orientació assolida en ambdós casos és la mateixa
    e2 = R1.euler()
    Ra = e2[0].R()
    Rb = e2[1].R()
    # R1.print()
    # Ra.print()
    # Rb.print()
    a = abs((Ra - R1).det())
    b = abs((Rb - R1).det())
    if  a < 0.0001 and b < 0.0001:
        print('TEST PASSED')
    else:
        print('TEST FAILED')


if __name__ == "__main__":
    test_degenerate1()
    test_degenerate2()
    test_euler()

