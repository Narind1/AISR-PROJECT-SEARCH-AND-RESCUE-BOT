"""
Referee for the search-and-rescue swarm.
- reads ../../config.json (n_robots, n_victims, comm, range, seed, time_limit, auto_quit, label)
- removes unused e-pucks, places victims at seeded random spots
- a victim counts as rescued when a robot stands still within 0.25 m of it for 1 s;
  the victim is then removed from the world
- stops when all victims are rescued or the time limit is reached, and appends one
  row to results/results.csv
"""
import os
import csv
import json
import math
import random
from controller import Supervisor

sup = Supervisor()
ts = int(sup.getBasicTimeStep())
dt = ts / 1000.0

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, "..", ".."))
cfg = {"label": "manual", "n_robots": 5, "n_victims": 3, "comm": True, "range": 1.0,
       "seed": 1, "time_limit": 500, "auto_quit": False}
try:
    with open(os.path.join(PROJECT, "config.json")) as f:
        cfg.update(json.load(f))
except Exception as e:
    print("config.json not read, using defaults:", e)

CANDIDATES = [(-0.75, 0.75), (0.0, 0.7), (0.7, 0.75), (0.75, 0.0), (-0.8, -0.05), (0.5, 0.5)]
RESCUE_DIST, RESCUE_SPEED, RESCUE_DWELL = 0.25, 0.01, 1.0

# ---- collect nodes by name
children = sup.getRoot().getField("children")
robots, victims = [], []
for i in range(children.getCount()):
    node = children.getMFNode(i)
    nf = node.getField("name")
    if nf is None:
        continue
    nm = nf.getSFString()
    if nm.startswith("epuck"):
        robots.append((int(nm[5:]), nm, node))
    elif nm.startswith("victim"):
        victims.append(node)

# ---- configure the run
n_robots, n_victims = int(cfg["n_robots"]), min(int(cfg["n_victims"]), len(CANDIDATES))
active_robots = []
for idx, nm, node in robots:
    if idx >= n_robots:
        node.remove()
    else:
        active_robots.append((nm, node))

spots = random.Random(int(cfg["seed"])).sample(CANDIDATES, n_victims)
active_victims = []
for k, node in enumerate(victims):
    if k < n_victims:
        node.getField("translation").setSFVec3f([spots[k][0], spots[k][1], 0.04])
        active_victims.append(node)
    else:
        node.remove()

print(f"[referee] run '{cfg['label']}': robots={n_robots}, victims={n_victims}, "
      f"comm={cfg['comm']}, range={cfg['range']}, seed={cfg['seed']}")

# ---- main loop
last_pos = {}
dwell = {}
rescue_times, rescue_by = [], {nm: 0 for nm, _ in active_robots}
finished = False


def finish(success, t):
    global finished
    finished = True
    t_all = rescue_times[-1] if success else float(cfg["time_limit"])
    row = {
        "label": cfg["label"], "seed": cfg["seed"], "n_robots": n_robots,
        "comm": int(bool(cfg["comm"])), "range": cfg["range"], "n_victims": n_victims,
        "rescued": len(rescue_times), "success": int(success),
        "t_first": round(rescue_times[0], 1) if rescue_times else "",
        "t_all": round(t_all, 1), "time_limit": cfg["time_limit"],
        "rescues_per_robot": ";".join(f"{k}:{v}" for k, v in rescue_by.items()),
    }
    os.makedirs(os.path.join(PROJECT, "results"), exist_ok=True)
    path = os.path.join(PROJECT, "results", "results.csv")
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if new:
            w.writeheader()
        w.writerow(row)
    print(f"[referee] finished: rescued {row['rescued']}/{n_victims} in {row['t_all']} s -> results.csv")
    if cfg.get("auto_quit"):
        sup.simulationQuit(0)
    else:
        sup.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)


while sup.step(ts) != -1:
    if finished:
        continue
    t = sup.getTime()

    speeds, positions = {}, {}
    for nm, node in active_robots:
        pos = node.getPosition()
        positions[nm] = pos
        prev = last_pos.get(nm)
        speeds[nm] = math.hypot(pos[0] - prev[0], pos[1] - prev[1]) / dt if prev else 1.0
        last_pos[nm] = pos

    for v in list(active_victims):
        vp = v.getPosition()
        best, best_d = None, 1e9
        for nm, pos in positions.items():
            d = math.hypot(pos[0] - vp[0], pos[1] - vp[1])
            if d < RESCUE_DIST and speeds[nm] < RESCUE_SPEED and d < best_d:
                best, best_d = nm, d
        vid = v.getId()
        dwell[vid] = dwell.get(vid, 0.0) + dt if best else 0.0
        if best and dwell[vid] >= RESCUE_DWELL:
            v.remove()
            active_victims.remove(v)
            rescue_times.append(t)
            rescue_by[best] += 1
            print(f"[referee] {best} rescued a victim at t={t:.1f}s "
                  f"({len(rescue_times)}/{n_victims})")

    if not active_victims:
        finish(True, t)
    elif t >= float(cfg["time_limit"]):
        finish(False, t)
