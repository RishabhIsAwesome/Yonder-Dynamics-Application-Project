"""
Ground-truth rover trajectory.

The rover drives a circular arc: radius 10m, constant forward speed 0.5 m/s.
This internal truth is never published to any topic — candidates only see
the noisy encoder ticks and GPS estimates derived from it.

Coordinate frame: local ENU (East-North-Up).
  x = east (metres from start)
  y = north (metres from start)
  heading = angle from east, counter-clockwise positive (radians)

The rover starts at (0, 0) heading north (π/2 rad).
"""

import math
import time


RADIUS_M = 10.0       # circle radius (metres)
SPEED_M_S = 0.5       # forward speed (metres / second)
ANGULAR_RATE = SPEED_M_S / RADIUS_M   # rad / second


class GroundTruth:
    """
    Computes the rover's true position and heading at any wall-clock time.

    The trajectory is a circle: the rover starts at the bottom of the circle
    and drives counter-clockwise.
      x(t) = RADIUS * sin(omega * t)
      y(t) = RADIUS * (1 - cos(omega * t))
    so at t=0 we have (0, 0) and the rover moves in the +y direction first.
    """

    def __init__(self):
        self._start_time = time.time()

    def state_at(self, t: float | None = None):
        """
        Returns (x, y, heading, speed) at wall-clock time t (default: now).

        x, y     metres in local ENU frame
        heading  radians (0 = east, π/2 = north)
        speed    m/s (constant)
        """
        if t is None:
            t = time.time()
        elapsed = t - self._start_time
        angle = ANGULAR_RATE * elapsed          # how far around the circle

        x = RADIUS_M * math.sin(angle)
        y = RADIUS_M * (1.0 - math.cos(angle))

        # Heading: tangent to the circle, rotating counter-clockwise
        heading = math.pi / 2.0 + angle        # starts north, rotates CCW

        return x, y, heading, SPEED_M_S

    def distance_in_interval(self, dt: float) -> float:
        """True distance travelled in dt seconds."""
        return SPEED_M_S * dt
