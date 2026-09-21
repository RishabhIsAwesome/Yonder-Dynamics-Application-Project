from dataclasses import dataclass, field

from simclock import now as _now


@dataclass
class Header:
    stamp: float = field(default_factory=_now)  # seconds since epoch (simulator clock, never jumps back)
    frame_id: str = ""
