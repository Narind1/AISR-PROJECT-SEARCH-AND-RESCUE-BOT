Search-and-Rescue Robot Swarm with Reactive Behaviours (Webots + e-puck + Python)

FOLDER
  worlds/sar_swarm.wbt            collapsed-building arena, 8 e-pucks, 6 candidate victims, referee
  controllers/epuck_sar/          robot controller (reflexive + procedural + declarative layers)
  controllers/sar_supervisor/     referee: removes rescued victims, times the run, writes results.csv
  config.json                     run settings (robots, victims, comm on/off, radio range, seed)
  run_experiments.py              batch experiments A (comm), B (swarm size), C (radio range)
  analyze.py                      summary table + plots from results/results.csv

RUN ONE SIMULATION (demo / video)
  1. Webots > File > Open World... > worlds/sar_swarm.wbt, press Play.
  2. Change settings by editing config.json, then reload the world (reload button).
  3. When all victims are rescued the referee prints the result and pauses the simulation.

RUN ALL EXPERIMENTS
  pip install matplotlib numpy
  python run_experiments.py          (about 50 runs; add a number for fewer seeds, e.g. "python run_experiments.py 3")
  python analyze.py
  If webots is not found: set WEBOTS_EXE to the full path of webots.exe first.

TROUBLESHOOTING
  - Robots drive in circles around a shared target: set YAW_SIGN = -1 in controllers/epuck_sar/epuck_sar.py
  - Victims not detected: print red_frac in the controller loop and tune the colour thresholds in detect_red()
  - World will not load (version error): change R2025a in worlds/sar_swarm.wbt to your Webots version
  - Texture "Cannot download" warnings are harmless (visual only)
