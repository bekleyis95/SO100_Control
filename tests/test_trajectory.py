import numpy as np
from robot_control.trajectory import Trajectory

def test_interpolation():
    start = np.zeros(6)
    end = np.ones(6)
    steps = 10
    trajectory = Trajectory.interpolate(start, end, steps)
    assert trajectory.shape == (steps, 6)
