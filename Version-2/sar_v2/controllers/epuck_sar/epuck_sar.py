"""
Search-and-Rescue swarm controller for the e-puck (Webots, Python).

Layered behaviour hierarchy (highest priority first):
  REFLEXIVE   1. Rescue pause (stop on a victim)
              2. Victim approach (camera sees red)
              3. Obstacle avoidance (proximity sensors) + stuck escape
  PROCEDURAL  4. Go-to-target: drive to a victim location received by radio (GPS + heading)
              5. Wander (random exploration)
  DECLARATIVE    Shared knowledge base `targets` = victim locations learned from other robots

Radio messages (Emitter/Receiver), comma separated:
  FOUND,<name>,<x>,<y>     a victim was seen at (x, y)
  RESCUED,<name>,<x>,<y>   the victim at (x, y) was reached; others forget it
Settings (comm on/off, radio range) are read from ../../config.json.
"""
import os
import json
import math
import random
from controller import Robot

try:
    import numpy as np
except ImportError:          # works without numpy, just slower
    np = None

# ---------------- parameters ----------------
MAX_SPEED = 6.28
OBSTACLE_THRESHOLD = 90
RED_CLOSE_FRACTION = 0.20
RESCUE_PAUSE = 1.5           # s stopped next to a victim
IGNORE_RED_AFTER_RESCUE = 3.0
ARRIVE_DIST = 0.15           # m: reached a shared target
TARGET_TTL = 120.0           # s: forget old targets
FORGET_RADIUS = 0.35         # m: RESCUED message removes targets this close
MERGE_RADIUS = 0.30          # m: FOUND messages closer than this are the same victim
CAL_TIME = 1.0               # s of straight driving to calibrate heading
YAW_SIGN = 1                 # set to -1 if robots circle around shared targets
WANDER_TURN_PROB = 0.3

# ---------------- config ----------------
HERE = os.path.dirname(os.path.abspath(__file__))
cfg = {"comm": True, "range": 1.0}
try:
    with open(os.path.join(HERE, "..", "..", "config.json")) as f:
        cfg.update(json.load(f))
except Exception as e:
    print("config.json not read, using defaults:", e)
COMM_ON = bool(cfg["comm"])
COMM_RANGE = float(cfg["range"])

robot = Robot()
name = robot.getName()
ts = int(robot.getBasicTimeStep())
random.seed(sum(map(ord, name)) + int(cfg.get("seed", 1)) * 1000)

# ---------------- devices ----------------
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

ps = []
for i in range(8):
    s = robot.getDevice(f"ps{i}")
    s.enable(ts)
    ps.append(s)

camera = robot.getDevice("camera")
camera.enable(ts)
cam_w, cam_h = camera.getWidth(), camera.getHeight()

gps = robot.getDevice("gps")
gps.enable(ts)
imu = robot.getDevice("imu")
imu.enable(ts)

emitter = robot.getDevice("radio_tx")
receiver = robot.getDevice("radio_rx")
if COMM_ON:
    emitter.setRange(COMM_RANGE if COMM_RANGE > 0 else -1)
    receiver.enable(ts)


# ---------------- helpers ----------------
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def set_speed(l, r):
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, l)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, r)))


def detect_red():
    """Return (red_pixel_fraction, normalised centroid x in [0,1])."""
    img = camera.getImage()
    if img is None:
        return 0.0, 0.5
    if np is not None:
        a = np.frombuffer(img, np.uint8).reshape((cam_h, cam_w, 4))   # BGRA
        mask = (a[:, :, 2] > 150) & (a[:, :, 1] < 90) & (a[:, :, 0] < 90)
        count = int(mask.sum())
        if count == 0:
            return 0.0, 0.5
        cols = np.nonzero(mask)[1]
        return count / (cam_w * cam_h), float(cols.mean()) / cam_w
    count, sum_x, samples = 0, 0, 0
    for xx in range(0, cam_w, 2):
        for yy in range(0, cam_h, 2):
            samples += 1
            if (camera.imageGetRed(img, cam_w, xx, yy) > 150 and
                    camera.imageGetGreen(img, cam_w, xx, yy) < 90 and
                    camera.imageGetBlue(img, cam_w, xx, yy) < 90):
                count += 1
                sum_x += xx
    if count == 0:
        return 0.0, 0.5
    return count / samples, (sum_x / count) / cam_w


def send(msg):
    if COMM_ON:
        emitter.send(msg.encode("utf-8"))


# ---------------- state ----------------
targets = []                 # declarative knowledge: [{"x":..,"y":..,"t":..}]
pause_until = -1e9
ignore_red_until = -1e9
last_found_send = -1e9
wander_dir, wander_steps = 0, 0
avoid_steps, escape_steps, escape_dir = 0, 0, 1
rescues = 0
yaw_offset = 0.0
calibrated = False
cal_start = None

# ---------------- main loop ----------------
while robot.step(ts) != -1:
    now = robot.getTime()
    g = gps.getValues()
    x, y = g[0], g[1]
    yaw = imu.getRollPitchYaw()[2]

    # ---- heading calibration: drive straight, compare GPS motion with IMU yaw
    if not calibrated:
        if cal_start is None and not (math.isnan(x) or math.isnan(y)):
            cal_start = (x, y)
        if now < CAL_TIME:
            set_speed(0.6 * MAX_SPEED, 0.6 * MAX_SPEED)
            continue
        if cal_start is not None and math.hypot(x - cal_start[0], y - cal_start[1]) > 0.03:
            gps_heading = math.atan2(y - cal_start[1], x - cal_start[0])
            yaw_offset = wrap(gps_heading - YAW_SIGN * yaw)
        else:
            print(f"[{name}] heading calibration failed, using offset 0")
        calibrated = True
        print(f"[{name}] yaw offset = {yaw_offset:.2f} rad")
    heading = wrap(YAW_SIGN * yaw + yaw_offset)

    # ---- communication
    if COMM_ON:
        while receiver.getQueueLength() > 0:
            parts = receiver.getString().split(",")
            receiver.nextPacket()
            if len(parts) != 4 or parts[1] == name:
                continue
            kind, tx, ty = parts[0], float(parts[2]), float(parts[3])
            if kind == "FOUND":
                for t in targets:
                    if math.hypot(t["x"] - tx, t["y"] - ty) < MERGE_RADIUS:
                        t["x"], t["y"], t["t"] = tx, ty, now
                        break
                else:
                    targets.append({"x": tx, "y": ty, "t": now})
            elif kind == "RESCUED":
                targets[:] = [t for t in targets
                              if math.hypot(t["x"] - tx, t["y"] - ty) > FORGET_RADIUS]
    targets[:] = [t for t in targets if now - t["t"] < TARGET_TTL]

    # ---- sensors
    p = [s.getValue() for s in ps]
    front_left = p[7] + p[6]
    front_right = p[0] + p[1]
    obstacle_near = max(p[0], p[1], p[6], p[7]) > OBSTACLE_THRESHOLD
    red_frac, red_cx = detect_red()
    victim_visible = red_frac > 0.005 and now > ignore_red_until

    # ---- Layer 1: rescue pause
    if now < pause_until:
        set_speed(0, 0)
        continue

    # ---- Layer 2: victim approach / rescue
    if victim_visible:
        close = red_frac > RED_CLOSE_FRACTION or (max(p[0], p[7]) > 300 and red_frac > 0.03)
        if close:
            rescues += 1
            pause_until = now + RESCUE_PAUSE
            ignore_red_until = now + RESCUE_PAUSE + IGNORE_RED_AFTER_RESCUE
            set_speed(0, 0)
            send(f"RESCUED,{name},{x:.3f},{y:.3f}")
            print(f"[{name}] reached a victim at ({x:.2f},{y:.2f}) t={now:.1f}s")
            continue
        if now - last_found_send > 0.5:
            send(f"FOUND,{name},{x:.3f},{y:.3f}")
            last_found_send = now
        err = red_cx - 0.5
        base = 0.6 * MAX_SPEED
        set_speed(base + 6.0 * err, base - 6.0 * err)
        continue

    # ---- Layer 3: obstacle avoidance (with stuck escape)
    if escape_steps > 0:
        escape_steps -= 1
        set_speed(escape_dir * 0.7 * MAX_SPEED, -escape_dir * 0.7 * MAX_SPEED)
        continue
    if obstacle_near:
        avoid_steps += 1
        if avoid_steps > 30:
            escape_steps, escape_dir, avoid_steps = 20, random.choice([-1, 1]), 0
        if front_left > front_right:
            set_speed(0.7 * MAX_SPEED, -0.7 * MAX_SPEED)     # turn right
        else:
            set_speed(-0.7 * MAX_SPEED, 0.7 * MAX_SPEED)     # turn left
        continue
    avoid_steps = 0

    # ---- Layer 4: go to the nearest shared victim location
    if targets:
        t = min(targets, key=lambda q: math.hypot(q["x"] - x, q["y"] - y))
        dist = math.hypot(t["x"] - x, t["y"] - y)
        if dist < ARRIVE_DIST:
            targets.remove(t)        # arrived, nothing red visible: forget it
        else:
            err = wrap(math.atan2(t["y"] - y, t["x"] - x) - heading)   # >0: target on the left
            base = 0.7 * MAX_SPEED * max(0.0, math.cos(err))
            turn = 3.0 * err
            set_speed(base - turn, base + turn)
            continue

    # ---- Layer 5: wander
    if wander_steps <= 0:
        if random.random() < WANDER_TURN_PROB:
            wander_dir = random.choice([-1, 1])
            wander_steps = random.randint(5, 20)
        else:
            wander_dir = 0
            wander_steps = random.randint(10, 40)
    wander_steps -= 1
    base = 0.8 * MAX_SPEED
    set_speed(base - wander_dir * 2.0, base + wander_dir * 2.0)
