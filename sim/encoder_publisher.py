"""
Simulated wheel encoder publisher.

Publishes WheelTicks messages on /wheel_ticks at ~50 Hz using BEST_EFFORT QoS.

Noise injected (representative of real encoder/communication issues):

  1. DROPPED TICKS (~1.5% per message)
     The cumulative tick counter is decremented by 1 before publishing.
     This simulates a hardware encoder missing a physical pulse due to
     vibration or electrical noise. Effect: position slowly undershoots
     ground truth. Over ~60 seconds this produces ~8cm of uncorrected drift.

  2. DUPLICATE MESSAGES (~2% of messages)
     The same (tick_count, timestamp) is published twice within ~5ms.
     This simulates a communication retransmit or interrupt double-fire.
     Effect on naive code: velocity = delta_ticks / delta_time → division
     by near-zero dt → infinite/NaN velocity reported.

  3. CLOCK DRIFT (continuous)
     Timestamps drift relative to wall clock via a slow cumulative random
     walk (Gaussian, σ=0.5ms per step). This simulates the independent
     clock on the encoder microcontroller drifting from the host.
     Effect on naive code: velocity estimates become less accurate over time.

QoS: BEST_EFFORT — a subscriber using default RELIABLE will silently receive
nothing. This is intentional and mirrors real rover behaviour.
"""

import math
import random
import threading
import time

from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ground_truth import GroundTruth
from messages import WheelTicks

# Matches the constants given in the assessment task description
WHEEL_RADIUS_M = 0.075
TICKS_PER_REVOLUTION = 360
DIST_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS_M) / TICKS_PER_REVOLUTION

PUBLISH_HZ = 50.0
PUBLISH_PERIOD = 1.0 / PUBLISH_HZ

# Noise parameters
DROPPED_TICK_PROB = 0.015     # probability of losing 1 tick per message
DUPLICATE_MSG_PROB = 0.020    # probability of re-sending a message
CLOCK_DRIFT_SIGMA = 0.0005    # std dev of timestamp drift per step (seconds)


class EncoderPublisher:
    """
    A simulated encoder node that publishes to /wheel_ticks.

    Access the underlying rclpy Node via self.node.
    """

    def __init__(self, ground_truth: GroundTruth, faults: bool = True):
        self._truth = ground_truth
        self._faults = faults

        self.node = Node("encoder_publisher")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._pub = self.node.create_publisher(WheelTicks, "/wheel_ticks", qos)
        self._logger = self.node.get_logger()

        # Internal state
        self._cumulative_ticks = 0      # true cumulative tick count
        self._reported_ticks = 0        # possibly corrupted reported count
        self._clock_offset = 0.0        # accumulated timestamp drift
        self._last_time = time.time()

        # Spin in a background thread
        self._thread = threading.Thread(
            target=self._spin, name="encoder_publisher", daemon=True
        )
        self._thread.start()

    def _spin(self) -> None:
        while True:
            start = time.time()
            self._publish_once()
            elapsed = time.time() - start
            sleep_for = max(0.0, PUBLISH_PERIOD - elapsed)
            time.sleep(sleep_for)

    def _publish_once(self) -> None:
        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        # How many ticks should have fired in this interval?
        true_distance = self._truth.distance_in_interval(dt)
        true_delta = int(true_distance / DIST_PER_TICK)
        self._cumulative_ticks += true_delta

        # Apply noise
        reported_delta = true_delta
        will_duplicate = False

        if self._faults:
            # Dropped tick
            if random.random() < DROPPED_TICK_PROB:
                reported_delta = max(0, true_delta - 1)

            # Duplicate message flag
            will_duplicate = random.random() < DUPLICATE_MSG_PROB

            # Clock drift
            self._clock_offset += random.gauss(0.0, CLOCK_DRIFT_SIGMA)

        self._reported_ticks += reported_delta

        # Build and publish the message
        msg = WheelTicks(
            tick_count=self._reported_ticks,
            timestamp=now + self._clock_offset,
        )
        self._pub.publish(msg)

        # Duplicate: re-publish the exact same message after a short delay
        if will_duplicate:
            dup_msg = WheelTicks(
                tick_count=msg.tick_count,
                timestamp=msg.timestamp,
            )
            # Small delay to arrive "just after" the real message
            t = threading.Timer(0.003, self._pub.publish, args=[dup_msg])
            t.daemon = True
            t.start()
