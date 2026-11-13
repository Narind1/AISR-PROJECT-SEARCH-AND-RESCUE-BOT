"""
Batch experiments for the SAR swarm. Runs Webots in fast mode, once per setting and seed,
and appends every run to results/results.csv (written by the referee controller).

Usage (from the project folder):
    python run_experiments.py            # all experiments, 5 seeds each
    python run_experiments.py A          # only experiment A
    python run_experiments.py A B 3      # experiments A and B with 3 seeds

Set the Webots executable if it is not on PATH, e.g. on Windows:
    set WEBOTS_EXE=C:\\Program Files\\Webots\\msys64\\mingw64\\bin\\webots.exe
"""
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "config.json")
WORLD = os.path.join(HERE, "worlds", "sar_swarm.wbt")
WEBOTS = os.environ.get("WEBOTS_EXE") or shutil.which("webots") or "webots"
WALL_TIMEOUT = 1800      # seconds of real time allowed per run

# experiment id -> list of (label, config overrides)
EXPERIMENTS = {
    "A": [("A:comm=off", {"comm": False}),
          ("A:comm=on", {"comm": True, "range": 1.0})],
    "B": [(f"B:robots={n}", {"n_robots": n, "comm": True, "range": 1.0}) for n in (1, 3, 5, 8)],
    "C": [(f"C:range={r}", {"comm": True, "range": r, "n_robots": 5}) for r in (0.3, 0.6, 1.0, -1)],
}

args = sys.argv[1:]
seeds = 5
sel = []
for a in args:
    if a.isdigit():
        seeds = int(a)
    else:
        sel.append(a.upper())
sel = sel or list(EXPERIMENTS)

with open(CFG) as f:
    original = f.read()
base = json.loads(original)
base.update({"auto_quit": True, "n_victims": 3, "n_robots": 5, "comm": True, "range": 1.0})

try:
    for exp in sel:
        for label, over in EXPERIMENTS[exp]:
            for seed in range(1, seeds + 1):
                cfg = dict(base)
                cfg.update(over)
                cfg.update({"label": label, "seed": seed})
                with open(CFG, "w") as f:
                    json.dump(cfg, f, indent=2)
                print(f"running {label} seed={seed} ...", flush=True)
                try:
                    subprocess.run([WEBOTS, "--mode=fast", "--minimize", "--no-rendering",
                                    "--batch", WORLD], timeout=WALL_TIMEOUT)
                except subprocess.TimeoutExpired:
                    print("  timed out")
finally:
    with open(CFG, "w") as f:
        f.write(original)
print("done. See results/results.csv, then run: python analyze.py")
