Search-and-Rescue Robot Swarm (Webots + e-puck + Python)

RUN:
1. Install Webots (cyberbotics.com) and Python 3.
2. Unzip this folder.
3. Webots > File > Open World... > worlds/sar_swarm.wbt
4. Press Play.

TROUBLESHOOTING:
- Robots turn away from the beacon: set BEACON_TURN_SIGN = -1 in controllers/epuck_sar/epuck_sar.py
- Victims not detected: print red_frac and tune the colour thresholds in detect_red()
- World won't load (version error): change R2025a in worlds/sar_swarm.wbt (header and URLs) to your Webots version
- Python not found: Webots > Tools > Preferences > Python command
