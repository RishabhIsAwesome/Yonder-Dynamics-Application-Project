"""
Live matplotlib visualizer.

Launched by: python sim/launch.py --visualize

Top panel, the path (metres, x = east, y = north):
  ·· Gray dotted   Ground truth path (the circle the rover really drives)
  ●  Red scatter   Raw GPS readings
  -- Orange dashed Encoder-only dead reckoning: wheel distance, drawn in the
                   rover's TRUE heading. The visualizer knows the truth and you
                   do not, so this line shows only the encoder's own errors
                   (dropped ticks); your node also has to work out the heading.
  ─  Blue solid    Your fused /odom output
  ●  Black dot     Where the rover really is now

Bottom panel: distance from the truth (metres) over time for GPS, the encoders
and your /odom. Each point is measured against the truth at the moment that
message arrived.

The visualizer subscribes to:
  /wheel_ticks    (WheelTicks)       — for the encoder line
  /gps_estimate   (GPSEstimate)      — for the GPS dots
  /odom           (nav_msgs/Odometry) — your output
It uses BEST_EFFORT QoS on all subscriptions.
"""

import math
import sys
import os
import threading
from collections import deque

import matplotlib

# Prefer Tk (it ships with most Python installs) but do not insist on it: if it
# is missing let matplotlib pick any other GUI backend (Qt, macOS, ...).
try:
    import tkinter  # noqa: F401
    matplotlib.use("TkAgg")
except ImportError:
    pass
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
from simclock import now as sim_now

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

_NON_GUI_BACKENDS = {"agg", "pdf", "ps", "svg", "cairo", "pgf", "template"}


class _VisualizerNode(Node):
    """Internal node that subscribes to all relevant topics."""

    def __init__(self, truth: GroundTruth):
        super().__init__("visualizer")
        self._truth = truth
        self._lock = threading.Lock()

        # GPS scatter + error over time
        self.gps_x: deque = deque(maxlen=MAX_HISTORY)
        self.gps_y: deque = deque(maxlen=MAX_HISTORY)
        self.gps_err_t: deque = deque(maxlen=MAX_HISTORY)
        self.gps_err: deque = deque(maxlen=MAX_HISTORY)

        # Encoder dead reckoning in the TRUE heading (accumulated here)
        self.enc_x: deque = deque(maxlen=MAX_HISTORY)
        self.enc_y: deque = deque(maxlen=MAX_HISTORY)
        self.enc_err_t: deque = deque(maxlen=MAX_HISTORY)
        self.enc_err: deque = deque(maxlen=MAX_HISTORY)
        self._enc_pos_x = 0.0
        self._enc_pos_y = 0.0
        self._last_tick_count = None

        # Candidate's fused /odom
        self.odom_x: deque = deque(maxlen=MAX_HISTORY)
        self.odom_y: deque = deque(maxlen=MAX_HISTORY)
        self.odom_err_t: deque = deque(maxlen=MAX_HISTORY)
        self.odom_err: deque = deque(maxlen=MAX_HISTORY)

        self.create_subscription(WheelTicks, "/wheel_ticks", self._on_ticks, BEST_EFFORT)
        self.create_subscription(GPSEstimate, "/gps_estimate", self._on_gps, BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self._on_odom, BEST_EFFORT)

    def _elapsed_and_truth(self):
        t = sim_now()
        tx, ty, heading, _ = self._truth.state_at(t)
        return t - self._truth.start_time, tx, ty, heading

    def _on_ticks(self, msg: WheelTicks) -> None:
        with self._lock:
            if self._last_tick_count is None:
                self._last_tick_count = msg.tick_count
                return
            delta = msg.tick_count - self._last_tick_count
            if delta <= 0:                       # duplicate (or nothing new): ignore
                return
            self._last_tick_count = msg.tick_count
            elapsed, tx, ty, heading = self._elapsed_and_truth()
            dist = delta * DIST_PER_TICK
            self._enc_pos_x += dist * math.cos(heading)
            self._enc_pos_y += dist * math.sin(heading)
            self.enc_x.append(self._enc_pos_x)
            self.enc_y.append(self._enc_pos_y)
            self.enc_err_t.append(elapsed)
            self.enc_err.append(math.hypot(self._enc_pos_x - tx, self._enc_pos_y - ty))

    def _on_gps(self, msg: GPSEstimate) -> None:
        with self._lock:
            elapsed, tx, ty, _ = self._elapsed_and_truth()
            self.gps_x.append(msg.x)
            self.gps_y.append(msg.y)
            self.gps_err_t.append(elapsed)
            self.gps_err.append(math.hypot(msg.x - tx, msg.y - ty))

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            elapsed, tx, ty, _ = self._elapsed_and_truth()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.odom_x.append(x)
            self.odom_y.append(y)
            self.odom_err_t.append(elapsed)
            self.odom_err.append(math.hypot(x - tx, y - ty))


class Visualizer:
    """
    Live matplotlib window.

    Call run() from the main thread — it blocks until the window is closed
    or Ctrl-C is pressed.
    """

    def __init__(self, ground_truth: GroundTruth):
        self._truth = ground_truth
        self._vnode = _VisualizerNode(ground_truth)

    def _ground_truth_path(self, n_points: int = 300):
        """Return (xs, ys) for one lap of the ground truth circle."""
        period = 2.0 * math.pi / abs(ANGULAR_RATE)
        t0 = self._truth.start_time
        pts = [self._truth.state_at(t0 + f * period) for f in np.linspace(0.0, 1.0, n_points)]
        return [p[0] for p in pts], [p[1] for p in pts]

    def run(self) -> None:
        backend = matplotlib.get_backend().lower()
        if backend in _NON_GUI_BACKENDS:
            print(f"[visualizer] No window is available (matplotlib is using its '{backend}' backend).")
            print("             Most likely Python's tkinter is not installed:")
            print("               Ubuntu/Debian/WSL:  sudo apt install python3-tk")
            print("               Fedora:             sudo dnf install python3-tkinter")
            print("               macOS/Windows:      reinstall Python from python.org (it includes tkinter)")
            print("             On WSL you also need a display (Windows 11 has one built in).")
            print("             Or run without --visualize and use the printed monitoring output.")
            return
        try:
            self._run_inner()
        except Exception as exc:
            print(f"[visualizer] Error: {exc!r}")
            print("             If this is a display error on WSL, make sure a display server")
            print("             (WSLg on Windows 11, or VcXsrv) is running, or run without --visualize.")

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
        enc_line, = ax_path.plot([], [], "--", color="darkorange", linewidth=1.2,
                                 label="Encoders only (true heading)")
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
        enc_err_line, = ax_err.plot([], [], "--", color="darkorange", linewidth=1.0, label="Encoders-only error (m)")
        odom_err_line, = ax_err.plot([], [], "-", color="royalblue", linewidth=1.5, label="Your /odom error (m)")
        ax_err.set_xlabel("Time (s)")
        ax_err.set_ylabel("Distance from truth (m)")
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
                if vn.gps_x:
                    gps_scatter.set_offsets(np.column_stack([list(vn.gps_x), list(vn.gps_y)]))
                if vn.enc_x:
                    enc_line.set_data(list(vn.enc_x), list(vn.enc_y))
                if vn.odom_x:
                    odom_line.set_data(list(vn.odom_x), list(vn.odom_y))

                tx, ty, _, _ = self._truth.state_at()
                rover_dot.set_data([tx], [ty])

                gps_err_line.set_data(list(vn.gps_err_t), list(vn.gps_err))
                enc_err_line.set_data(list(vn.enc_err_t), list(vn.enc_err))
                odom_err_line.set_data(list(vn.odom_err_t), list(vn.odom_err))

                t_now = sim_now() - self._truth.start_time
                ax_err.set_xlim(0, max(60.0, t_now))
                tallest = max([1.5] + [max(e) * 1.1 for e in (vn.gps_err, vn.enc_err, vn.odom_err) if e])
                ax_err.set_ylim(0, tallest)

                odom_status = (
                    f"  /odom: ({vn.odom_x[-1]:.2f}, {vn.odom_y[-1]:.2f}) m"
                    if vn.odom_x else "  /odom: waiting (no messages yet)"
                )
                status_text.set_text(f"GPS readings: {len(vn.gps_x)}   |   {odom_status}")

            return gps_scatter, enc_line, odom_line, rover_dot, gps_err_line, enc_err_line, odom_err_line, status_text

        ani = animation.FuncAnimation(
            fig, update, interval=100, blit=False, cache_frame_data=False
        )

        plt.tight_layout()
        plt.show()
