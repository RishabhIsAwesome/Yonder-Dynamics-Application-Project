from dataclasses import dataclass, field
import time as _time


@dataclass
class Header:
    stamp: float = field(default_factory=_time.time)  # seconds since epoch
    frame_id: str = ""
