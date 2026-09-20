# Embedded Take-Home: Embedded Systems Track

## Part 0: Base Knowledge Primer

### What is ROS 2?

ROS 2 (Robot Operating System 2) is a framework for writing robot software as a collection of independent programs called **nodes** that talk to each other by passing messages. Instead of one giant program controlling everything, you have small focused programs — one for motor control, one for GPS, one for camera processing — that communicate over named channels called **topics**.

### Topics, Publishers, Subscribers

- A **topic** is a named channel (e.g., `/wheel_ticks`) that carries a specific message type.
- A **publisher** sends messages onto a topic.
- A **subscriber** receives messages from a topic.
- Nodes don't know about each other directly — they agree only on a topic name and message type.

### QoS (Quality of Service)

ROS 2 lets you configure delivery guarantees per topic: `RELIABLE` vs `BEST_EFFORT`.

- **`RELIABLE`** guarantees every message is delivered. The publisher retransmits until acknowledged.
- **`BEST_EFFORT`** sends once and moves on, spending no resources tracking delivery.

**Critical:** a publisher and subscriber on the same topic must have *compatible* QoS settings, or they silently won't connect — no error, no messages, just silence. This trips people up constantly, including AI code generators, which often default to mismatched settings. Compatibility rule: the subscriber's reliability level must be ≤ the publisher's (a BEST_EFFORT publisher cannot serve a RELIABLE subscriber).

### TF2 (Transform Tree)

TF2 tracks position/orientation relationships between reference frames on the robot. Transforms have a **parent frame** and a **child frame** — getting this order backwards is one of the most common ROS bugs (including in AI-generated code). A `odom → base_link` transform describes `base_link` *relative to* `odom`.

### EKF and Sensor Fusion

An Extended Kalman Filter combines multiple noisy sensor sources (e.g., wheel encoders + GPS) into a single best estimate of position, weighting each by how much you trust it at that moment. You won't implement a full EKF here, but you will do a simplified version of exactly this fusion problem.

### How Yonder actually uses this

Our rover fuses wheel encoder data with RTK GPS through a TF2 transform tree (`map → odom → base_link`) to know where it is.

- `base_link` is the rover's body frame (centered on the rover, essentially fixed).
- `odom` is the rover's position relative to its start point. The `odom → base_link` transform comes from wheel encoders: where is the rover now relative to where it started?
- `map` is the world frame. The `map → odom` transform comes from GPS: it corrects the odom frame's drift by anchoring it to an absolute reference.

The encoder data comes from six ODrive motor controllers via CAN bus (`odrive_can/ControllerStatus.pos_estimate`). GPS comes from dual NMEA receivers publishing `NavSatFix`. A `robot_localization` EKF fuses everything into `/autonomous/localization/odometry/global`. This task is a simplified version of that pipeline.

**Resources:**
- [ROS 2 Publisher/Subscriber tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 QoS docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [TF2 introduction](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)

---

## Starter Repo

### Getting the repo

You need [Git](https://git-scm.com/downloads). The repo is public, so no account or login is needed:

```bash
git clone https://gitlab.com/Yonder-Dynamics/take-home-projects/embedded-take-home.git
cd embedded-take-home
```

Don't want to use Git? Download [a ZIP of the repo](https://gitlab.com/Yonder-Dynamics/take-home-projects/embedded-take-home/-/archive/main/embedded-take-home-main.zip), unzip it, and work in the `embedded-take-home-main` folder. A ZIP has no Git history, so the "Submitting" section below has one extra step for you.

### What's in the repo

```
odometry_node.py       Your file — stub with function signatures, docstrings,
                       and TODO comments. This is what you'll implement.

sim/
  launch.py            Entry point. Run this to start everything.
  encoder_publisher.py Simulates wheel encoder ticks with realistic noise.
                       PROVIDED — working, do not modify.
  gps_publisher.py     Simulates GPS position estimates with noise and outages.
                       PROVIDED — working, do not modify.
  ground_truth.py      The rover's true trajectory (circular arc). Internal to
                       the simulator — you never see this directly, only the
                       noisy sensor readings derived from it.
  messages.py          WheelTicks and GPSEstimate message definitions.
  visualizer.py        Live matplotlib plot. Run with --visualize.
  rclpy_lite/          A lightweight simulator shim with the same API as real rclpy.
                       Lets you write ROS-style code without installing ROS.
  nav_msgs/            Standard ROS message types (Odometry, etc.) as dataclasses.
  geometry_msgs/       Pose, Twist, Quaternion, etc.
  tf2_ros/             TransformBroadcaster stub for Stretch Goal A.

requirements.txt       pip dependencies (numpy, matplotlib only).
AI_LOG.md              Template for your AI usage log (Part 3).
METHODOLOGY.md         Template for how to run your code and your thought process (Part 4).
```

### Files you shouldn't edit

We run your node against our own copy of the simulator, so changes to these won't carry over, and they can make your node behave differently for us than for you:

- Everything in `sim/` (the simulator, message definitions and ROS shims).
- The constants at the top of `odometry_node.py` (`WHEEL_RADIUS_M`, `TICKS_PER_REVOLUTION`, `DIST_PER_TICK`). They match the real rover's encoder setup.

Your work goes in `odometry_node.py`. You can add new files of your own next to it, and update `requirements.txt`.

### Familiarisation — read these before you start

Before touching `odometry_node.py`, read through the files you're given:

**`sim/encoder_publisher.py`** — This is the most important file to read first.
It publishes `WheelTicks` messages on `/wheel_ticks`. Note:
- The QoS profile it uses (hint: this is the thing most likely to silently break your node).
- The three types of noise it injects: dropped ticks, duplicate messages, and clock drift.
  Each has a comment explaining the real-world phenomenon it simulates.

**`sim/gps_publisher.py`** — Publishes `GPSEstimate` on `/gps_estimate`.
Note the QoS, the update rate, and the outage cycle. The `covariance` field in each
message tells you how much to trust that reading.

**`sim/messages.py`** — Defines `WheelTicks` and `GPSEstimate`. Read the docstrings
carefully — every field is described.

**`sim/rclpy_lite/`** — You don't need to understand this in depth. It's a lightweight
shim with the same API as real rclpy (Node, Publisher, Subscriber, QoS, spin). We name
it `rclpy_lite` rather than `rclpy` to avoid ambiguity if you have ROS installed.
To port your code to a real ROS 2 system, swap `rclpy_lite` → `rclpy` in your imports.

**`sim/ground_truth.py`** — The rover drives a circle (radius 10m, 0.5 m/s). You won't
subscribe to this topic — only `encoder_publisher` and `gps_publisher` use it internally.
It's provided so you can understand what the true trajectory looks like.

### Running it

You need Python 3.10 or newer and no other dependencies beyond numpy and matplotlib.

1. **Install dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

2. **Start the simulator** (from the repo root):

   ```bash
   python sim/launch.py
   ```

   This starts the encoder and GPS publishers, and attempts to load your
   `odometry_node.py`. Until you implement the subscriptions, your node will
   start silently and publish nothing.

3. **Start with clean feeds** while you get the basics working:

   ```bash
   python sim/launch.py --no-faults
   ```

   `--no-faults` disables all injected noise — GPS is clean, no duplicate
   messages, no dropped ticks. Good for verifying your subscriptions connect
   before tackling the noise.

4. **Enable the live visualizer** once you have `/odom` publishing:

   ```bash
   python sim/launch.py --visualize
   ```

   Opens a matplotlib window showing ground truth, GPS readings, encoder-only
   dead reckoning, and your fused `/odom` output in real time. The blue line
   only appears once your node is publishing — it's a visual reward for getting
   the core task right.

| Command | What it does |
| --- | --- |
| `python sim/launch.py` | Default: noisy feeds |
| `python sim/launch.py --no-faults` | Clean feeds while building basics |
| `python sim/launch.py --visualize` | Adds live matplotlib plot |
| `python sim/launch.py --no-node` | Run simulator only, no odometry node |

### The feeds

| Topic | Rate | Message | QoS |
| --- | --- | --- | --- |
| `/wheel_ticks` | ~50 Hz | `WheelTicks{tick_count: int, timestamp: float}` | check the source |
| `/gps_estimate` | ~1 Hz | `GPSEstimate{x: float, y: float, timestamp: float, covariance: float}` | check the source |

- `tick_count` is a cumulative total — it only ever increases (or stays the same on a duplicate).
- `x` and `y` are in metres, in a local ENU frame where (0, 0) is the rover's start position.
- `covariance` is in m² — it tells you the GPS reading's uncertainty, useful for fusion weighting.

### Things that go wrong on purpose

The feeds misbehave in ways representative of real hardware. With `--no-faults` off:

**Wheel encoder:**
- **Duplicate messages**: the same `(tick_count, timestamp)` pair arrives twice within a few milliseconds. Naive velocity calculation (`delta_ticks / delta_time`) gives a near-zero denominator — NaN or infinite velocity.
- **Dropped ticks**: the cumulative counter occasionally falls 1 behind ground truth. Each individual drop is tiny (~1.3mm), but they accumulate into a slow drift. GPS fusion is your primary correction for this.
- **Clock drift**: timestamps drift slowly from wall clock via a random walk. Velocity estimates become less accurate over time.

**GPS:**
- Gaussian position noise (σ ≈ 0.3m in normal conditions).
- ~5s outages repeating every ~45s cycle — the feed goes silent then resumes.
- Elevated noise (σ ≈ 0.8m) for ~10s after each outage resumes (satellite reacquisition).
- The `covariance` field reflects the actual σ² at each moment.

Deciding how to detect and handle each of these is part of the task.

---

## Part 1: Core Task (required)

Complete `odometry_node.py` so that it:

1. **Subscribes to `/wheel_ticks`** and converts tick count changes into distance and velocity using the provided constants:
   ```
   WHEEL_RADIUS_M       = 0.075    # metres
   TICKS_PER_REVOLUTION = 360
   DIST_PER_TICK        = (2π × WHEEL_RADIUS) / TICKS_PER_REVOLUTION  ≈ 0.00131 m
   ```

2. **Handles the injected noise gracefully.** Dropped and duplicate ticks should not silently corrupt your distance estimate or crash your node. Document your approach to detecting and handling each in your `METHODOLOGY.md`.

3. **Subscribes to `/gps_estimate`** and performs a simple fusion between your wheel-derived position and the GPS estimate. A basic weighted average based on which source you trust more at a given moment is sufficient. A full EKF is not required.

4. **Publishes the fused result** as a `nav_msgs/Odometry` message on `/odom`. Check `sim/nav_msgs/msg/__init__.py` and the [real ROS 2 nav_msgs/Odometry spec](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html) for the field layout.

5. **Provides monitoring output** once per second to the terminal showing:
   - Time since last message from each source
   - Measured receive rate for `/wheel_ticks` vs expected ~50 Hz
   - Current fused position (x, y)

### What we're looking for

- Does it run against the provided scaffold and produce a sensible fused position estimate?
- Does it visibly handle the injected noise (we will be able to tell from your output whether dropped/duplicate ticks corrupted your result)?
- Correct QoS configuration — your node actually connects to the provided publishers.
- A completed `METHODOLOGY.md` that lets us run your code, and explains your fusion approach, your noise-handling logic, and any design decisions.

---

## Part 2: Stretch Goals (optional)

Pick any/all. Partial, well-reasoned attempts are valued over none.

**A. TF2 broadcaster**

Publish your fused odometry as a proper TF2 transform (`odom → base_link`) instead of just a message. Use the provided `TransformBroadcaster` in `sim/tf2_ros/`. Pay close attention to parent/child frame order and quaternion conventions — this is a common spot where things look right but are backwards.

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
```

**B. Confidence-weighted fusion**

Instead of a fixed weighting between wheel and GPS estimates, make the weighting dynamic — e.g., trust wheel odometry more over short time windows and GPS more as wheel-derived drift accumulates. The `covariance` field on `GPSEstimate` messages gives you the GPS uncertainty at each moment. This is conceptually close to what a real EKF does.

**C. Full TF2 (advanced)**

Implement a full `map → odom → base_link` transform system using the published data. Create the `odom → base_link` transform from encoder data, and a `map → odom` correction from GPS. Ensure that querying the rover's `map` position between GPS updates returns a sensible interpolated result.

---

## Part 3: AI Usage Log (required)

Submit a short `AI_LOG.md` with your code (there is a template in the repo root). For each significant use of AI tools, note:

- What you asked
- What you kept vs. rewrote, and why
- Anything the AI got wrong that you had to catch
- How you verified it actually worked correctly (not just that it compiled)

This is not graded on whether you used AI — it is graded on whether you can tell us what it got wrong and why you fixed it.

---

## Part 4: METHODOLOGY.md (required)

Edit the `METHODOLOGY.md` in the repo root (there is a template) so it covers:

- **How to run your code.** The exact steps for a reviewer to install the dependencies and run your node against the simulator from a fresh clone and see it working.
- **Your thought process, in bullet points.** Why you built it the way you did: your fusion approach, how you detect and handle the noise, the calls you made on ambiguous parts, and how you tested it.

We read this alongside your code. Write it in your own words: we'd rather see clear reasoning and honest limitations than a polished description.

Keep your `requirements.txt` up to date. It must list every library your code needs.

---

## Rubric

| Criterion | What we're scoring |
| --- | --- |
| **Correctness** | Core task runs against the scaffold, produces sensible fused odometry, pub/sub actually connects |
| **Noise handling** | Dropped/duplicate ticks and noisy GPS are detected and handled, not silently ignored |
| **Design judgment** | Evidence of intentional choices beyond the minimum (fusion weighting, code structure, sensible defaults) |
| **Handling ambiguity** | How did they resolve underspecified parts of the task? Did they make a reasonable call and explain it? |
| **Understanding, not just output** | Can they explain their own code/math? Does `METHODOLOGY.md` show real comprehension? |
| **AI verification** | Evidence they tested/verified AI-assisted code rather than taking it on faith (from log + code quality) |
| **Stretch engagement** (bonus) | Attempted or completed any stretch goal — even partial attempts count positively |

We don't expect a perfect implementation. Those who show genuine effort and learning are the ones who will have a leg up!

---

## Submitting

1. **Create a public repository on your own GitHub account.**
2. **Point your clone at it.** Your clone's `origin` is our repo, which you can't push to:

   ```bash
   git remote set-url origin https://github.com/<your-username>/<your-repo>.git
   git push -u origin main
   ```

   If you downloaded the ZIP instead of cloning, there is no `origin` yet. Start a repo and add yours:

   ```bash
   git init -b main
   git add .
   git commit -m "Initial commit"
   git remote add origin https://github.com/<your-username>/<your-repo>.git
   git push -u origin main
   ```

3. **Check that it's public.** Open your repo's link in a private/incognito browser window. If you can see the code without logging in, so can we.
4. **Send us the link** in the Google Form you'll be asked to fill out.

Your repo should include your code, your completed `METHODOLOGY.md`, and your `AI_LOG.md`.
