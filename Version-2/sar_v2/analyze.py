"""Summarise results/results.csv into a table and plots (needs matplotlib)."""
import csv
import os
import statistics as st
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(HERE, "results", "results.csv")
rows = list(csv.DictReader(open(path)))
groups = defaultdict(list)
for r in rows:
    groups[r["label"]].append(r)


def num(v):
    return float(v) if v not in ("", None) else float("nan")


summary = []
print(f"{'setting':<18}{'runs':>5}{'success%':>10}{'mean t_all (s)':>16}{'std':>8}{'mean t_first':>14}")
for label in sorted(groups):
    g = groups[label]
    t_all = [num(r["t_all"]) for r in g]
    t_first = [num(r["t_first"]) for r in g if r["t_first"] != ""]
    succ = 100.0 * sum(int(r["success"]) for r in g) / len(g)
    sd = st.pstdev(t_all) if len(t_all) > 1 else 0.0
    mf = st.mean(t_first) if t_first else float("nan")
    summary.append((label, len(g), succ, st.mean(t_all), sd, mf))
    print(f"{label:<18}{len(g):>5}{succ:>10.0f}{st.mean(t_all):>16.1f}{sd:>8.1f}{mf:>14.1f}")

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib not installed: pip install matplotlib to get plots")
    raise SystemExit

out = os.path.join(HERE, "results")


def plot(prefix, title, xlabel, fname, numeric=False):
    items = [s for s in summary if s[0].startswith(prefix)]
    if not items:
        return
    labels = [s[0].split("=")[1] for s in items]
    if numeric:
        order = sorted(range(len(items)), key=lambda i: float(labels[i]) if float(labels[i]) > 0 else 99)
        items, labels = [items[i] for i in order], [labels[i] for i in order]
        labels = ["inf" if l == "-1" else l for l in labels]
    fig, ax = plt.subplots(figsize=(5.5, 3.6))
    ax.bar(labels, [s[3] for s in items], yerr=[s[4] for s in items], capsize=4)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Time to rescue all victims (s)")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(os.path.join(out, fname), dpi=200)
    plt.close(fig)


plot("A:", "Communication on vs off", "communication", "exp_A_comm.png")
plot("B:", "Effect of swarm size", "number of robots", "exp_B_size.png", numeric=True)
plot("C:", "Effect of radio range", "radio range (m)", "exp_C_range.png", numeric=True)
print("plots saved in results/")
