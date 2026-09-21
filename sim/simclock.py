"""
The simulator's clock.

Every timestamp the simulator produces comes from `now()`. It starts at the
real wall-clock time (seconds since the Unix epoch) when the simulator starts,
then advances with time.monotonic(), so it never jumps backwards.

Why not just time.time()? The system wall clock can be stepped by NTP or by a
virtual machine syncing with its host. On WSL2 we have seen it jump back by more
than a second in the middle of a run, which made the encoder report negative
distances. Real rovers have the same problem, which is why robotics code
measures durations with a monotonic clock.

Consequence for your node: use time.monotonic() to measure how long ago
something happened. Do not compute `time.time() - msg.timestamp`.
"""

import time

_WALL_AT_START = time.time()
_MONO_AT_START = time.monotonic()


def now() -> float:
    """Seconds since the Unix epoch, on a clock that only moves forward."""
    return _WALL_AT_START + (time.monotonic() - _MONO_AT_START)
