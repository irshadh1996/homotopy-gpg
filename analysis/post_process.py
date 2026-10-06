
import os
import sys
import json
import itertools
import numpy as np
import matplotlib.pyplot as plt

GOAL_TOL = 0.01          # m, an agent counts as arrived inside this
BAND     = (0.35, 0.40)  # m, conflict band for dwell time


def load_metrics(path):
    goals, columns = None, None
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            text = line[1:].strip()
            if text.startswith("goals="):
                goals = np.array(json.loads(text[len("goals="):]), dtype=float)
            elif text.startswith("iter,"):
                columns = text.split(",")
    data = np.loadtxt(path, delimiter=",", comments="#", ndmin=2)
    return goals, columns, data


def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_dir = os.path.join(repo_root, "results", "homotopy_guided_games", "scenario_manual")
    scen_dir = sys.argv[1] if len(sys.argv) > 1 else default_dir
    met_path = os.path.join(scen_dir, "metrics_agent0.txt")

    goals, columns, data = load_metrics(met_path)
    n = len(goals)
    t = data[:, columns.index("t_rel")]
    P = np.stack([data[:, [columns.index(f"x{j}"), columns.index(f"y{j}")]]
                  for j in range(n)], axis=1)          # (T, n, 2)

    # ── completion: first time each agent is within GOAL_TOL of its goal ─────
    d_goal = np.linalg.norm(P - goals[None, :, :], axis=2)   # (T, n)
    arrived = d_goal < GOAL_TOL
    if arrived.any(axis=0).all():
        arrival_idx = arrived.argmax(axis=0)
        end = int(arrival_idx.max()) + 1
        completion_time = t[end - 1] - t[0]
    else:
        end = len(t)
        completion_time = np.nan

    # Metrics below are computed over the active window [start, completion]
    Pw, tw = P[:end], t[:end]

    # ── path length per agent ────────────────────────────────────────────────
    path_len = np.linalg.norm(np.diff(Pw, axis=0), axis=2).sum(axis=0)   # (n,)

    # ── pairwise center-to-center distances ──────────────────────────────────
    pairs = list(itertools.combinations(range(n), 2))
    D = np.stack([np.linalg.norm(Pw[:, i] - Pw[:, j], axis=1) for i, j in pairs], axis=1)
    min_clearance = D.min()

    # ── dwell time: time any pair spends inside the conflict band ────────────
    in_band = ((D >= BAND[0]) & (D <= BAND[1])).any(axis=1)
    dwell = np.diff(tw)[in_band[:-1]].sum()

    # ── statistics ───────────────────────────────────────────────────────────
    print(f"\nScenario: {scen_dir}")
    print(f"Agents: {n}")
    print("-" * 50)
    print(f"Completion time [s]          : {completion_time:.2f}")
    for j in range(n):
        print(f"Path length robot {j+1} [m]     : {path_len[j]:.3f}")
    print(f"Path length mean ± std [m]   : {path_len.mean():.3f} ± {path_len.std():.3f}")
    print(f"Min clearance [m]            : {min_clearance:.3f}")
    print(f"Dwell time in [{BAND[0]}, {BAND[1]}] m [s]: {dwell:.2f}")
    print("-" * 50)

    # ── trajectory plot ──────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(6, 6))
    for j in range(n):
        line, = ax.plot(P[:, j, 0], P[:, j, 1], label=f"Robot {j+1}")
        ax.plot(P[0, j, 0], P[0, j, 1], "o", color=line.get_color())
        ax.plot(goals[j, 0], goals[j, 1], "*", markersize=14, color=line.get_color())
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("Trajectories (o = start, * = goal)")
    fig.tight_layout()
    out_png = os.path.join(scen_dir, "trajectories.png")
    fig.savefig(out_png, dpi=200)
    print(f"Saved {out_png}")
    plt.show()


if __name__ == "__main__":
    main()