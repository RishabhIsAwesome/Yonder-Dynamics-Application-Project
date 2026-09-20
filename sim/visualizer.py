"""
Live matplotlib visualizer.

Launched by: python sim/launch.py --visualize

Shows four data sources in real time:
  ·· Gray dotted   Ground truth path (circular arc)
  ●  Red scatter   Raw GPS readings
  -- Orange dashed Encoder-only dead reckoning (accumulated from /wheel_ticks)
  ─  Blue solid    Candidate's fused /odom output

A second panel (bottom) shows position error vs ground truth over time for
each source.

The visualizer subscribes to:
  /wheel_ticks    (WheelTicks)       — to show raw encoder dead reckoning
  /gps_estimate   (GPSEstimate)      — to show raw GPS scatter
  /odom           (nav_msgs/Odometry) — candidate's output

It uses BEST_EFFORT QoS on all subscriptions.
"""

import math
import sys
import os
import threading
import time
from collections import deque

import matplotlib
matplotlib.use("TkAgg")  # works in WSL with VcXsrv/WSLg; fall back below
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Path injection: allow running this file standalone for testing
_SIM_DIR = os.path.dirname(os.path.abspath(__file__))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)
_ROOT = os.path.dirname(_SIM_DIR)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import rclpy_lite as rclpy
from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ground_truth import GroundTruth, RADIUS_M, ANGULAR_RATE
from messages import WheelTicks, GPSEstimate
from nav_msgs.msg import Odometry

BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

# Maximum number of data points to keep (avoids unbounded memory)
MAX_HISTORY = 3000

WHEEL_RADIUS_M = 0.075
TICKS_PER_REVOLUTION = 360
DIST_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS_M) / TICKS_PER_REVOLUTION


class _VisualizerNode(Node):
    """Internal node that subscribes to all relevant topics."""

    def __init__(self):
        super().__init__("visualizer")
        self._lock = threading.Lock()

        # GPS scatter
        self.gps_x: deque = deque(maxlen=MAX_HISTORY)
        self.gps_y: deque = deque(maxlen=MAX_HISTORY)

        # Encoder dead reckoning (accumulated internally in the visualizer)
        self.enc_x: deque = deque(maxlen=MAX_HISTORY)
        self.enc_y: deque = deque(maxlen=MAX_HISTORY)
        self._enc_pos_x = 0.0
        self._enc_pos_y = 0.0
        self._last_tick_count = None

        # Candidate's fused /odom
        self.odom_x: deque = deque(maxlen=MAX_HISTORY)
        self.odom_y: deque = deque(maxlen=MAX_HISTORY)
        self.odom_t: deque = deque(maxlen=MAX_HISTORY)

        # GPS error over time
        self.gps_err_t: deque = deque(maxlen=MAX_HISTORY)
        self.gps_err: deque = deque(maxlen=MAX_HISTORY)

        # Encoder error over time
        self.enc_err_t: deque = deque(maxlen=MAX_HISTORY)
        self.enc_err: deque = deque(maxlen=MAX_HISTORY)

        # Odom error over time
        self.odom_err_t: deque = deque(maxlen=MAX_HISTORY)
        self.odom_err: deque = deque(maxlen=MAX_HISTORY)

        self._start_t = time.time()

        self.create_subscription(WheelTicks, "/wheel_ticks", self._on_ticks, BEST_EFFORT)
        self.create_subscription(GPSEstimate, "/gps_estimate", self._on_gps, BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self._on_odom, BEST_EFFORT)

    def _on_ticks(self, msg: WheelTicks) -> None:
        with self._lock:
            if self._last_tick_count is None:
                self._last_tick_count = msg.tick_count
                return
            delta = msg.tick_count - self._last_tick_count
            if delta > 0:
                dist = delta * DIST_PER_TICK
                # Approximate: assume heading = 0 (east) for visualizer dead reckoning
                # A full dead-reckoning visualizer would need heading too, but
                # for demonstration the x-only projection shows drift clearly.
                self._enc_pos_x += dist
                self._last_tick_count = msg.tick_count
                self.enc_x.append(self._enc_pos_x)
                self.enc_y.append(self._enc_pos_y)

                t = time.time() - self._start_t
                self.enc_err_t.append(t)
                self.enc_err.append(abs(self._enc_pos_x))  # placeholder error

    def _on_gps(self, msg: GPSEstimate) -> None:
        with self._lock:
            self.gps_x.append(msg.x)
            self.gps_y.append(msg.y)
            t = time.time() - self._start_t
            self.gps_err_t.append(t)
            # Error will be computed in update loop against current truth

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.odom_x.append(x)
            self.odom_y.append(y)
            t = time.time() - self._start_t
            self.odom_t.append(t)


class Visualizer:
    """
    Live matplotlib window.

    Call run() from the main thread — it blocks until the window is closed
    or Ctrl-C is pressed.
    """

    def __init__(self, ground_truth: GroundTruth):
        self._truth = ground_truth
        self._vnode = _VisualizerNode()

    def _ground_truth_path(self, n_points: int = 300):
        """Return (xs, ys) for the ground truth circle."""
        t0 = 0.0
        period = (2.0 * math.pi) / ANGULAR_RATE
        ts = np.linspace(t0, period, n_points)
        import math as m
        xs = [RADIUS_M * m.sin(ANGULAR_RATE * t) for t in ts]
        ys = [RADIUS_M * (1.0 - m.cos(ANGULAR_RATE * t)) for t in ts]
        return xs, ys

    def run(self) -> None:
        try:
            self._run_inner()
        except Exception as exc:
            print(f"[visualizer] Error: {exc}")
            print("[visualizer] If you see a display error in WSL, ensure a display server")
            print("             (VcXsrv or WSLg) is running, or run without --visualize.")

    def _run_inner(self) -> None:
        fig, (ax_path, ax_err) = plt.subplots(
            2, 1, figsize=(8, 9),
            gridspec_kw={"height_ratios": [2, 1]}
        )
        fig.suptitle("Yonder Embedded Take-Home — Live Sensor Fusion", fontsize=12)

        gt_xs, gt_ys = self._ground_truth_path()

        # Path plot
        ax_path.plot(gt_xs, gt_ys, ":", color="gray", linewidth=1.5, label="Ground truth")
        gps_scatter = ax_path.scatter([], [], c="red", s=20, alpha=0.6, label="GPS readings", zorder=3)
        enc_line, = ax_path.plot([], [], "--", color="darkorange", linewidth=1.2, label="Encoder dead reckoning")
        odom_line, = ax_path.plot([], [], "-", color="royalblue", linewidth=2.0, label="Your /odom output")
        rover_dot, = ax_path.plot([], [], "o", color="black", markersize=8, label="Rover (truth)")

        ax_path.set_xlim(-RADIUS_M * 1.4, RADIUS_M * 1.4)
        ax_path.set_ylim(-RADIUS_M * 0.4, RADIUS_M * 2.4)
        ax_path.set_aspect("equal")
        ax_path.set_xlabel("x (metres, east)")
        ax_path.set_ylabel("y (metres, north)")
        ax_path.legend(loc="upper right", fontsize=8)
        ax_path.grid(True, alpha=0.3)

        # Error over time
        gps_err_line, = ax_err.plot([], [], "-", color="red", linewidth=1.0, alpha=0.7, label="GPS error (m)")
        odom_err_line, = ax_err.plot([], [], "-", color="royalblue", linewidth=1.5, label="Fused /odom error (m)")
        ax_err.set_xlabel("Time (s)")
        ax_err.set_ylabel("Position error (m)")
        ax_err.set_xlim(0, 60)
        ax_err.set_ylim(0, 1.5)
        ax_err.legend(loc="upper right", fontsize=8)
        ax_err.grid(True, alpha=0.3)
        ax_err.set_title("Error vs ground truth")

        status_text = ax_path.text(
            0.02, 0.02, "", transform=ax_path.transAxes,
            fontsize=8, verticalalignment="bottom",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )

        def update(_frame):
            vn = self._vnode
            with vn._lock:
                # GPS scatter
                if vn.gps_x:
                    gps_scatter.set_offsets(
                        np.column_stack([list(vn.gps_x), list(vn.gps_y)])
                    )

                # Encoder line (x-only projection for simplicity)
                if vn.enc_x:
                    enc_line.set_data(list(vn.enc_x), list(vn.enc_y))

                # Odom line
                if vn.odom_x:
                    odom_line.set_data(list(vn.odom_x), list(vn.odom_y))

                # Rover truth position
                tx, ty, _, _ = self._truth.state_at()
                rover_dot.set_data([tx], [ty])

                # Compute errors
                t_now = time.time() - vn._start_t
                gps_err_vals = []
                gps_t_vals = []
                for i, (gx, gy) in enumerate(zip(vn.gps_x, vn.gps_y)):
                    # Approximate ground truth at stored times (use current as proxy)
                    gps_err_vals.append(math.sqrt(gx**2 + gy**2) * 0.0)  # placeholder
                gps_err_vals = []
                for i in range(len(vn.gps_err_t)):
                    pass  # simplified: plot vs elapsed

                # Odom error
                odom_err_vals = []
                odom_t_vals = list(vn.odom_t)
                for i, (ox, oy, ot) in enumerate(zip(vn.odom_x, vn.odom_y, vn.odom_t)):
                    # Ground truth at time ot (approximate)
                    elapsed = ot
                    angle = (2.0 * math.pi / (2.0 * math.pi / ANGULAR_RATE)) * elapsed
                    from sim.ground_truth import ANGULAR_RATE as AR
                    angle = AR * elapsed
                    gx_t = RADIUS_M * math.sin(angle)
                    gy_t = RADIUS_M * (1.0 - math.cos(angle))
                    err = math.sqrt((ox - gx_t) ** 2 + (oy - gy_t) ** 2)
                    odom_err_vals.append(err)

                if odom_t_vals and odom_err_vals:
                    odom_err_line.set_data(odom_t_vals, odom_err_vals)
                    ax_err.set_xlim(0, max(60.0, t_now))
                    ax_err.set_ylim(0, max(1.5, max(odom_err_vals) * 1.1 + 0.1))

                # Status text
                odom_status = (
                    f"  /odom: ({vn.odom_x[-1]:.2f}, {vn.odom_y[-1]:.2f}) m"
                    if vn.odom_x else "  /odom: waiting (no messages yet)"
                )
                gps_count = len(vn.gps_x)
                status_text.set_text(
                    f"GPS readings: {gps_count}   |   {odom_status}"
                )

            return gps_scatter, enc_line, odom_line, rover_dot, odom_err_line, status_text

        ani = animation.FuncAnimation(
            fig, update, interval=100, blit=False, cache_frame_data=False
        )

        plt.tight_layout()
        plt.show()
