import numpy as np
from scipy.interpolate import CubicSpline

class Trajectory:
    """
    Handles Cartesian-to-joint trajectory interpolation with cubic smoothing.
    """
    @staticmethod
    def interpolate(start: np.ndarray, end: np.ndarray, steps: int) -> np.ndarray:
        """
        Interpolate between start and end poses.
        :param start: Starting joint angles.
        :param end: Ending joint angles.
        :param steps: Number of interpolation steps.
        :return: Array of interpolated joint angles.
        """
        t = np.linspace(0, 1, steps)
        interpolator = CubicSpline([0, 1], np.vstack([start, end]), axis=0)
        return interpolator(t)
