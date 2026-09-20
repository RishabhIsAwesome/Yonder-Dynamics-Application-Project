"""
Launch script — entry point for the entire simulation.

    python sim/launch.py                # run simulator + candidate node
    python sim/launch.py --visualize    # same, plus live matplotlib plot
    python sim/launch.py --no-faults    # clean feeds (no noise/outages)
    python sim/launch.py --no-node      # run sim only, no candidate node

The script injects sim/ into sys.path so that:
  - `import rclpy_lite` resolves to sim/rclpy_lite/ (our ROS shim)
  - `from nav_msgs.msg import Odometry` resolves to sim/nav_msgs/
  - etc.
No ROS installation required.

Usage note: stop with Ctrl-C.
"""

import argparse
import os
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

import rclpy_lite as rclpy
from ground_truth import GroundTruth
from encoder_publisher import EncoderPublisher
from gps_publisher import GPSPublisher


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
    return p.parse_args()


def load_candidate_node():
    """Import and instantiate the candidate's OdometryNode."""
    try:
        from odometry_node import OdometryNode  # type: ignore
        node = OdometryNode()
        print("[launch] odometry_node.py loaded successfully.")
        return node
    except ImportError as exc:
        print(f"[launch] Warning: could not import odometry_node.py: {exc}")
        return None
    except Exception as exc:
        print(f"[launch] Error initialising OdometryNode: {exc}")
        import traceback
        traceback.print_exc()
        return None


def main():
    args = parse_args()
    faults = not args.no_faults

    if args.no_faults:
        print("[launch] Running with clean feeds (--no-faults).")

    rclpy.init()

    truth = GroundTruth()
    enc = EncoderPublisher(truth, faults=faults)
    gps = GPSPublisher(truth, faults=faults)

    print("[launch] Encoder publisher started  → /wheel_ticks   @ ~50 Hz  (BEST_EFFORT)")
    print("[launch] GPS publisher started       → /gps_estimate  @ ~1 Hz   (BEST_EFFORT)")
    print()

    odom_node = None
    if not args.no_node:
        odom_node = load_candidate_node()
        if odom_node is None:
            print(
                "[launch] Continuing without a candidate node.\n"
                "         Implement OdometryNode in odometry_node.py to see output."
            )

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
    """Start the executor in a thread, run matplotlib in the main thread."""
    from visualizer import Visualizer

    stop_event = threading.Event()

    def executor_thread():
        try:
            while not stop_event.is_set():
                time.sleep(0.02)
        except Exception:
            pass

    t = threading.Thread(target=executor_thread, daemon=True)
    t.start()

    vis = Visualizer(truth)
    try:
        vis.run()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
