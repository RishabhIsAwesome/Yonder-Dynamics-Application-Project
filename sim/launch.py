"""
Launch script — entry point for the entire simulation.

    python sim/launch.py                # run simulator + candidate node
    python sim/launch.py --visualize    # same, plus live matplotlib plot
    python sim/launch.py --no-faults    # clean feeds (no noise/outages)
    python sim/launch.py --no-node      # run sim only, no candidate node
    python sim/launch.py --seed 42      # repeat exactly the same noise

The script injects sim/ into sys.path so that:
  - `import rclpy_lite` resolves to sim/rclpy_lite/ (our ROS shim)
  - `from nav_msgs.msg import Odometry` resolves to sim/nav_msgs/
  - etc.
No ROS installation required.

Usage note: stop with Ctrl-C.
"""

import argparse
import os
import random
import sys
import threading
import time

# ── Path injection ──────────────────────────────────────────────────────────
# Insert the sim/ directory so that `import rclpy_lite`, `from nav_msgs.msg import …`
# etc. resolve to our shims instead of requiring a real ROS installation.
_SIM_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SIM_DIR)

# Also make the project root importable (for `import odometry_node`)
_PROJECT_ROOT = os.path.dirname(_SIM_DIR)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
# ────────────────────────────────────────────────────────────────────────────

# Some terminals and redirects (older Windows consoles, `> file.txt`) can't print every character.
# A stray symbol in a print() must never crash a run, so replace what can't be encoded.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass

import rclpy_lite as rclpy
from ground_truth import GroundTruth
from encoder_publisher import EncoderPublisher
from gps_publisher import GPSPublisher
from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy


def parse_args():
    p = argparse.ArgumentParser(description="Yonder embedded take-home simulator")
    p.add_argument(
        "--visualize", action="store_true",
        help="Open a live matplotlib window showing all data sources"
    )
    p.add_argument(
        "--no-faults", action="store_true",
        help="Disable all injected noise (clean feeds for initial development)"
    )
    p.add_argument(
        "--no-node", action="store_true",
        help="Run simulator only, skip loading odometry_node.py"
    )
    p.add_argument(
        "--seed", type=int, default=None,
        help="Seed the random noise so a run can be repeated exactly (default: random, printed at startup)"
    )
    return p.parse_args()


def load_candidate_node():
    """Import and instantiate the candidate's OdometryNode."""
    import traceback
    try:
        from odometry_node import OdometryNode  # type: ignore
        node = OdometryNode()
        print("[launch] odometry_node.py loaded successfully.")
        _warn_about_unconnected_subscriptions(node)
        return node
    except Exception as exc:
        print(f"[launch] Could not load OdometryNode from odometry_node.py: {exc!r}")
        traceback.print_exc()
        return None


def _warn_about_unconnected_subscriptions(node) -> None:
    """A subscription that did not connect (QoS mismatch) never receives a message."""
    for sub in getattr(node, "_subscriptions", []):
        if not sub.connected:
            print(
                f"[launch] WARNING: your subscription to {sub.topic} is NOT connected: its QoS "
                f"does not match the publisher's, so it will receive nothing. See the QoS "
                f"section of the README and sim/encoder_publisher.py."
            )


def _watch_for_odom(seconds: float = 5.0) -> None:
    """After a few seconds, say so if nothing has been published on /odom."""
    seen = []
    monitor = Node("launch_monitor")
    monitor.create_subscription(
        object, "/odom", lambda m: seen.append(1),
        QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10),
    )

    def check():
        if not seen:
            print(
                f"[launch] Note: nothing has been published on /odom after {seconds:.0f}s. "
                f"That is expected until you implement odometry_node.py. If you have, look "
                f"above for an 'Exception in ...' traceback or a QoS warning."
            )

    timer = threading.Timer(seconds, check)
    timer.daemon = True
    timer.start()


def main():
    args = parse_args()
    faults = not args.no_faults

    if args.no_faults:
        print("[launch] Running with clean feeds (--no-faults).")

    seed = args.seed if args.seed is not None else random.randrange(1_000_000)
    random.seed(seed)
    print(f"[launch] Random seed: {seed}  (rerun with --seed {seed} to repeat this exact noise)")

    rclpy.init()

    truth = GroundTruth()
    enc = EncoderPublisher(truth, faults=faults)
    gps = GPSPublisher(truth, faults=faults)

    print("[launch] Encoder publisher started  -> /wheel_ticks   @ ~50 Hz  (BEST_EFFORT)")
    print("[launch] GPS publisher started       -> /gps_estimate  @ ~1 Hz   (BEST_EFFORT)")
    print()

    odom_node = None
    if not args.no_node:
        odom_node = load_candidate_node()
        if odom_node is None:
            print(
                "[launch] Continuing without a candidate node.\n"
                "         Implement OdometryNode in odometry_node.py to see output."
            )
        else:
            _watch_for_odom()

    print()
    if args.visualize:
        _run_with_visualizer(truth, odom_node)
    else:
        print("[launch] Running. Press Ctrl-C to stop.")
        print("         Tip: run with --visualize to see a live plot.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    print("\n[launch] Shutting down.")
    rclpy.shutdown()


def _run_with_visualizer(truth: GroundTruth, odom_node) -> None:
    """Run matplotlib in the main thread (publishers and your node run in their own threads)."""
    from visualizer import Visualizer

    vis = Visualizer(truth)
    try:
        vis.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
