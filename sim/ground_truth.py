"""
Ground-truth rover trajectory.

The rover drives a circle: radius 10m, constant forward speed 0.5 m/s.
This internal truth is never published to any topic — candidates only see
the noisy encoder ticks and GPS estimates derived from it.

Coordinate frame: local ENU (East-North-Up).
  x = east (metres from start)
  y = north (metres from start)
  heading = direction the rover is facing, as an angle from east,
            counter-clockwise positive (radians)

The rover starts at (0, 0) heading EAST (0 rad), at the bottom of a circle whose
centre is (0, 10), and drives counter-clockwise (so it turns left, steadily).
"""

import math

from simclock import now as _now


RADIUS_M = 10.0       # circle radius (metres)
SPEED_M_S = 0.5       # forward speed (metres / second)
ANGULAR_RATE = SPEED_M_S / RADIUS_M   # rad / second (positive = turning left)


class GroundTruth:
    """
    Computes the rover's true position and heading at any time.

    Times are on the simulator clock (see simclock.py). The trajectory is a
    circle: the rover starts at the bottom of the circle and drives
    counter-clockwise.
      x(t) = RADIUS * sin(omega * t)
      y(t) = RADIUS * (1 - cos(omega * t))
    so at t=0 we have (0, 0) and the rover moves in the +x (east) direction first.
    """

    def __init__(self):
        self._start_time = _now()

    @property
    def start_time(self) -> float:
        """Simulator-clock time at which the rover was at (0, 0)."""
        return self._start_time

    def state_at(self, t: float | None = None):
        """
        Returns (x, y, heading, speed) at simulator-clock time t (default: now).

        x, y     metres in local ENU frame
        heading  radians (0 = east, π/2 = north); the direction of travel
        speed    m/s (constant)
        """
        if t is None:
            t = _now()
        elapsed = t - self._start_time
        angle = ANGULAR_RATE * elapsed          # how far around the circle

        x = RADIUS_M * math.sin(angle)
        y = RADIUS_M * (1.0 - math.cos(angle))

        # Heading: tangent to the circle. It starts at 0 (east) and grows at
        # ANGULAR_RATE, the same angle we have travelled around the circle.
        heading = angle

        return x, y, heading, SPEED_M_S

    def distance_in_interval(self, dt: float) -> float:
        """True distance travelled in dt seconds."""
        return SPEED_M_S * dt
