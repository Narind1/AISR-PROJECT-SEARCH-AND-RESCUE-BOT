"""
Search-and-Rescue swarm controller for the e-puck (Webots, Python).
Layered reactive behaviours (highest priority first):
  1. RESCUED  - park after reaching a victim
  2. Victim approach / rescue (camera sees red)
  3. Obstacle avoidance (proximity sensors)
  4. Follow a FOUND beacon from another robot (Emitter/Receiver)
  5. Wander
"""
from controller import Robot
import random
import math

MAX_SPEED = 6.28
OBSTACLE_THRESHOLD = 90      # proximity value that counts as "obstacle"
RED_CLOSE_FRACTION = 0.20    # fraction of red pixels => close enough to rescue
BEACON_TIMEOUT = 2.0         # seconds a FOUND beacon stays "fresh"
IGNORE_VICTIM_TIME = 15.0    # seconds to ignore red after someone else rescued
WANDER_TURN_PROB = 0.03
BEACON_TURN_SIGN = 1         # set to -1 if robots turn AWAY from the beacon

SEARCH, RESCUED = "SEARCH", "RESCUED"

robot = Robot()
name = robot.getName()
ts = int(robot.getBasicTimeStep())

# --- Devices -----------------------------------------------------------
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

emitter = robot.getDevice("radio_tx")
receiver = robot.getDevice("radio_rx")
receiver.enable(ts)


# --- Helpers -----------------------------------------------------------
def set_speed(l, r):
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, l)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, r)))


def detect_red():
    """Return (red_pixel_fraction, normalized_centroid_x in [0,1])."""
    img = camera.getImage()
    if img is None:
        return 0.0, 0.5
    count, sum_x = 0, 0
    for x in range(cam_w):
        for y in range(cam_h):
            r = camera.imageGetRed(img, cam_w, x, y)
            g = camera.imageGetGreen(img, cam_w, x, y)
            b = camera.imageGetBlue(img, cam_w, x, y)
            if r > 150 and g < 90 and b < 90:
                count += 1
                sum_x += x
    if count == 0:
        return 0.0, 0.5
    return count / (cam_w * cam_h), (sum_x / count) / cam_w


def send(msg):
    emitter.send(msg.encode("utf-8"))


# --- State -------------------------------------------------------------
state = SEARCH
beacon_angle = None
last_beacon_time = -1e9
ignore_victim_until = -1e9
last_found_send = -1e9
wander_dir = 0
wander_steps = 0

# --- Main loop ---------------------------------------------------------
while robot.step(ts) != -1:
    now = robot.getTime()

    # ---- Communication: read all pending packets
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        direction = receiver.getEmitterDirection()
        receiver.nextPacket()
        kind, _, sender = msg.partition(":")
        if sender == name:
            continue
        if kind == "FOUND" and state == SEARCH:
            beacon_angle = math.atan2(direction[1], direction[0])
            last_beacon_time = now
        elif kind == "RESCUED":
            last_beacon_time = -1e9
            ignore_victim_until = now + IGNORE_VICTIM_TIME

    # ---- Sensors
    p = [s.getValue() for s in ps]
    front_left = p[7] + p[6]
    front_right = p[0] + p[1]
    obstacle_near = max(p[0], p[1], p[6], p[7]) > OBSTACLE_THRESHOLD
    red_frac, red_cx = detect_red()
    victim_visible = red_frac > 0.005 and now > ignore_victim_until

    # ---- Layer 1: rescued -> park
    if state == RESCUED:
        set_speed(0, 0)
        continue

    # ---- Layer 2: victim approach / rescue
    if victim_visible:
        if red_frac > RED_CLOSE_FRACTION or max(p[0], p[7]) > 300:
            state = RESCUED
            set_speed(0, 0)
            send(f"RESCUED:{name}")
            print(f"[{name}] rescued a victim at t={now:.1f}s")
            continue
        if now - last_found_send > 0.5:
            send(f"FOUND:{name}")
            last_found_send = now
        err = red_cx - 0.5          # >0: victim to the right
        base = 0.6 * MAX_SPEED
        set_speed(base + 6.0 * err, base - 6.0 * err)
        continue

    # ---- Layer 3: obstacle avoidance
    if obstacle_near:
        if front_left > front_right:
            set_speed(0.7 * MAX_SPEED, -0.7 * MAX_SPEED)   # turn right
        else:
            set_speed(-0.7 * MAX_SPEED, 0.7 * MAX_SPEED)   # turn left
        continue

    # ---- Layer 4: follow a fresh beacon
    if beacon_angle is not None and now - last_beacon_time < BEACON_TIMEOUT:
        base = 0.7 * MAX_SPEED
        turn = BEACON_TURN_SIGN * max(-1.0, min(1.0, beacon_angle)) * 3.0
        set_speed(base - turn, base + turn)
        continue

    # ---- Layer 5: wander
    if wander_steps <= 0:
        if random.random() < WANDER_TURN_PROB * 10:
            wander_dir = random.choice([-1, 1])
            wander_steps = random.randint(5, 20)
        else:
            wander_dir = 0
            wander_steps = random.randint(10, 40)
    wander_steps -= 1
    base = 0.8 * MAX_SPEED
    set_speed(base - wander_dir * 2.0, base + wander_dir * 2.0)
