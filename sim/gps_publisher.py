"""
Simulated GPS publisher.

Publishes GPSEstimate messages on /gps_estimate at ~1 Hz using BEST_EFFORT QoS.

The GPS provides an absolute position in the local ENU frame (metres from start),
derived from ground truth with added Gaussian noise.

Noise injected (representative of degraded RTK-GPS conditions):

  1. GAUSSIAN POSITION NOISE
     Each reading has independent x and y noise: N(0, σ²) where σ ≈ 0.3m.
     The msg.covariance field reflects the true σ² at that moment, which
     candidates can use for fusion weighting.

  2. OUTAGES (~5 second gaps, repeating on a 45s cycle)
     Mirrors real GPS signal loss (overpass, multipath). The feed goes
     silent — no messages — then resumes. The monitoring output candidates
     build should make this visible.

  3. ELEVATED NOISE AFTER OUTAGE
     For ~10s after an outage, covariance is raised (σ ≈ 0.8m) to simulate
     the receiver reacquiring satellites. Covariance gradually returns to
     normal.

QoS: BEST_EFFORT — same as the real rover's GPS topic.
"""

import math
import random
import threading
import time

from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ground_truth import GroundTruth
from messages import GPSEstimate

PUBLISH_HZ = 1.0
PUBLISH_PERIOD = 1.0 / PUBLISH_HZ

# Noise parameters
NOISE_NORMAL_SIGMA = 0.30      # m, good conditions
NOISE_DEGRADED_SIGMA = 0.80    # m, post-outage re-acquisition

# Outage cycle (seconds)
OUTAGE_CYCLE = 45.0            # repeat every 45s
OUTAGE_DURATION = 5.0          # gap length
DEGRADED_DURATION = 10.0       # elevated noise after each outage


class GPSPublisher:
    """
    A simulated GPS node that publishes to /gps_estimate.

    Access the underlying rclpy Node via self.node.
    """

    def __init__(self, ground_truth: GroundTruth, faults: bool = True):
        self._truth = ground_truth
        self._faults = faults

        self.node = Node("gps_publisher")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._pub = self.node.create_publisher(GPSEstimate, "/gps_estimate", qos)
        self._logger = self.node.get_logger()

        self._run_start = time.time()

        self._thread = threading.Thread(
            target=self._spin, name="gps_publisher", daemon=True
        )
        self._thread.start()

    def _spin(self) -> None:
        while True:
            start = time.time()
            self._publish_once()
            elapsed = time.time() - start
            time.sleep(max(0.0, PUBLISH_PERIOD - elapsed))

    def _publish_once(self) -> None:
        now = time.time()
        phase = (now - self._run_start) % OUTAGE_CYCLE

        if self._faults:
            # During outage window: publish nothing
            if phase >= (OUTAGE_CYCLE - OUTAGE_DURATION):
                return

            # Choose noise level: degraded for DEGRADED_DURATION after outage resumes
            time_since_outage_end = phase  # outage resets at phase=0
            if time_since_outage_end < DEGRADED_DURATION:
                sigma = NOISE_DEGRADED_SIGMA
            else:
                sigma = NOISE_NORMAL_SIGMA
        else:
            sigma = NOISE_NORMAL_SIGMA

        x_true, y_true, _, _ = self._truth.state_at(now)

        msg = GPSEstimate(
            x=x_true + random.gauss(0.0, sigma),
            y=y_true + random.gauss(0.0, sigma),
            timestamp=now,
            covariance=sigma ** 2,
        )
        self._pub.publish(msg)
