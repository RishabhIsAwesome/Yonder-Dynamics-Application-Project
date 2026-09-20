"""
Custom message types used in this simulation.

These are not real ROS message types — they are purpose-built for this
take-home. In a real system, encoder ticks would come from hardware-specific
messages (e.g., odrive_can/ControllerStatus).
"""

from dataclasses import dataclass


@dataclass
class WheelTicks:
    """
    Published by the wheel encoder simulator at ~50 Hz.

    Fields:
        tick_count  Cumulative encoder tick count. Increases monotonically as
                    the rover moves forward. Subject to noise (see README).
        timestamp   Wall-clock time when this reading was taken, in seconds
                    since the Unix epoch. Subject to slow clock drift.
    """
    tick_count: int = 0
    timestamp: float = 0.0


@dataclass
class GPSEstimate:
    """
    Published by the GPS simulator at ~1 Hz.

    Position is in metres in a local ENU (East-North-Up) frame.
    (0, 0) is the rover's starting position.

    Fields:
        x           East position in metres.
        y           North position in metres.
        timestamp   Wall-clock time of this reading (seconds).
        covariance  Estimated position uncertainty in m² (1-sigma²).
                    Higher = less trustworthy reading. Use this when
                    deciding how much to weight GPS in your fusion.
    """
    x: float = 0.0
    y: float = 0.0
    timestamp: float = 0.0
    covariance: float = 0.09  # ~0.3m std dev in good conditions
