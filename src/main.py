import time
import sys
import os
import pickle
import json
import itertools
import numpy as np
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from acados_template import AcadosOcpSolver
from utilities.acados_solver_param import acados_solver_model
from utilities.websocket_manager import RoboWebsocketManager
from utilities import keyboard_listener
from utilities import scalar_rank
from utilities.robot_mover import reset_world, move_robots
from utilities.config_files import ROBOT_CONFIGS, N_players, CMD_VEL_MESSAGE_TYPE, ODOM_MESSAGE_TYPE
from utilities.path_helpers import compute_robot_homotopies_updated, pad_traj_to_length

METHOD_NAME = "homotopy_guided_games"

def solver_creator(ocp):
    return AcadosOcpSolver(ocp, json_file='acados_ocp.json', build=True)


def append_jsonl(path, record):
    with open(path, "a") as f:
        f.write(json.dumps(record, default=float) + "\n")

if __name__ == "__main__":
    is_batch = len(sys.argv) > 3
    instance_idx = sys.argv[3] if is_batch else "manual"

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    base_dir  = os.path.join(repo_root, "results", "homotopy_guided_games")
    log_dir  = os.path.join(base_dir, "log")
    pkl_dir  = os.path.join(base_dir, "pkl")
    met_dir  = os.path.join(base_dir, f"scenario_{instance_idx}")
    for d in [log_dir, pkl_dir, met_dir]:
        os.makedirs(d, exist_ok=True)

    pkl_path = os.path.join(pkl_dir, f"data_{instance_idx}.pkl")
    txt_path = os.path.join(log_dir, f"data_{instance_idx}.txt")
    met_path = os.path.join(met_dir, "metrics_agent0.txt")

    listener = keyboard_listener.start_keyboard_listener()

    if is_batch:
        start_positions = np.array(json.loads(sys.argv[1]))
        x_goals         = np.array(json.loads(sys.argv[2]))
    else:
        start_positions = np.array([[-1.5, 0], [1.5, 0], [0, -1.5]])
        x_goals         = np.array([[ 1.5, 0], [-1.5, -0], [0, 1.5]])

    reset_world(); time.sleep(1.0)
    move_robots(start_positions, x_goals)

    ws_manager = RoboWebsocketManager(ROBOT_CONFIGS, N_players, CMD_VEL_MESSAGE_TYPE, ODOM_MESSAGE_TYPE)
    ws_manager.start()
    if not ws_manager.all_initial_received.wait(timeout=30):
        print("Failed to receive initial robot states. Exiting.")
        sys.exit(1)

    x0 = ws_manager.get_x0()
    current_robot_locations = x0.reshape(-1, 3)[:, :2]
    current_theta_0         = x0.reshape(-1, 3)[:, 2]

    [ocp, Tf, N_raw, o_radius, _, nx_raw, nu_total_players_raw] = acados_solver_model(
        N_players, current_robot_locations, x_goals, current_theta_0)
    N = int(N_raw[0] if isinstance(N_raw, (tuple, list, np.ndarray)) else N_raw)
    ocp_solver = solver_creator(ocp)
        # Number of top-ranked joint homotopy combinations solved per step, by number of players
    N_TOP_BY_PLAYERS = {3: 3, 4: 8, 5: 22}
    if N_players not in N_TOP_BY_PLAYERS:
        raise ValueError(f"No top-k value defined for N_players={N_players}; "
                         f"supported: {list(N_TOP_BY_PLAYERS)}")
    N_TOP = N_TOP_BY_PLAYERS[N_players]
    max_iterations = 770
    metadata = {
        "schema_version": "1.1",
        "method": METHOD_NAME,
        "instance_idx": instance_idx,
        "n_players": int(N_players),
        "sim_steps_max": max_iterations,
        "starts": start_positions.tolist(),
        "goals":  x_goals.tolist(),
        "N_horizon": int(N),
        "Tf": float(Tf),
        "obstacle_radius": float(o_radius),
        "timestamp_start": time.time(),
        "timestamp_start_human": time.ctime()
    }
    full_data = {"metadata": metadata, "step_data": []}

    with open(txt_path, "w") as f:
        f.write(json.dumps({"type": "metadata", **metadata}, default=float) + "\n")


    vel_ref_0  = np.array([0.22] * N_players)
    d_spacing  = vel_ref_0 * 0.1
    u0         = np.zeros((N, (N_players * 2)))
    x0_full    = None
    previous_best_hsignatures = None
    pos_cols = ",".join(f"x{j},y{j},th{j}" for j in range(N_players))
    metric_columns = (
        "iter,t_rel," + pos_cols +
        ",dist_to_goal_ego,goal_reached,best_v,best_omega,best_cost,"
        "n_candidates,n_valid,warmup,pub_ok,estop"
    )
    f_met = open(met_path, "w")
    f_met.write(f"# method={METHOD_NAME} instance={instance_idx} "
                f"N_players={N_players} n_top={N_TOP} N_range={max_iterations}\n")
    f_met.write(f"# r_agent={float(o_radius)}\n")
    f_met.write(f"# v_ref={float(vel_ref_0[0])} N_horizon={N} Tf={float(Tf)}\n")
    f_met.write(f"# goals={np.round(np.asarray(x_goals, dtype=float), 4).tolist()}\n")
    f_met.write("# t_rel is wall-clock seconds since the loop started; "
                "integrate metrics with the actual dt between rows.\n")
    f_met.write("# centralized method: no warmup phase, warmup=0 on every row.\n")
    f_met.write("# estop=1 rows had zero control published (safety stop).\n")
    f_met.write("# " + metric_columns + "\n")
    f_met.flush()

    t_sim_start    = time.perf_counter()
    last_dump_time = time.perf_counter()
    DUMP_INTERVAL_S = 5.0
    STEP_TIME_LIMIT_S = 0.10
    TEST_DELAY_S = 0.00  # artificial delay to test the fallback; set to 0.0 to disable

    for iteration in range(max_iterations):
        if ws_manager.shutdown_event.is_set():
            print("Shutdown event detected.")
            break

        iter_start = time.perf_counter()

        current_x0 = ws_manager.get_x0()
        if current_x0 is None:
            continue

        curr_states = [current_x0[i*3 : i*3+3].tolist() for i in range(N_players)]

        # 1) Warm start
        t_ws_start = time.perf_counter()
        ocp_solver.set(0, "lbx", current_x0)
        ocp_solver.set(0, "ubx", current_x0)
        if iteration > 0 and x0_full is not None:
            for stage in range(N):
                ocp_solver.set(stage, "x", x0_full[stage])
                ocp_solver.set(stage, "u", u0[stage])
        warm_start_duration = time.perf_counter() - t_ws_start

        # 2) Homotopy computation
        t_homotopy_start = time.perf_counter()
        with ProcessPoolExecutor(max_workers=N_players) as executor:
            func = partial(compute_robot_homotopies_updated,
                           robot_locations=current_robot_locations,
                           x_goals=x_goals,
                           o_radius=o_radius,
                           d_spacing=d_spacing)
            results = list(executor.map(func, range(N_players)))

        all_traj_lists   = [dict(results)[f"Robot_{i+1}"] for i in range(N_players)]
        all_combinations = list(itertools.product(*all_traj_lists))
        homotopy_duration = time.perf_counter() - t_homotopy_start

        # 3) Ranking
        t_ranking_start = time.perf_counter()
        ranked_combinations = []
        for idx, comb in enumerate(all_combinations):
            score, _ = scalar_rank.scalar_rank_combination(
                [t for (_, t) in comb], num_points=N + 1, proximity_threshold=0.30)
            ranked_combinations.append((idx, comb, score))
        ranked_combinations.sort(key=lambda x: x[2])
        ranking_duration = time.perf_counter() - t_ranking_start

        # 4) OCP — solve the top candidates, keep only the best
        t_ocp_batch_start = time.perf_counter()
        best = None
        n_solved = 0
        n_attempted = 0
        for i, (idx, comb, _score) in enumerate(ranked_combinations[:N_TOP]):
            n_attempted += 1
            candidate_hsigs = [hsig for (hsig, _) in comb]
            l_val = 0.0 if previous_best_hsignatures == candidate_hsigs else 1.0

            for stage in range(N + 1):
                p_val = np.concatenate([
                    [pt for j in range(N_players)
                        for pt in pad_traj_to_length(np.asarray(comb[j][1]), N + 1)[stage]],
                    [l_val]
                ])
                ocp_solver.set(stage, "p", p_val)

            status = ocp_solver.solve()
            if status <= 2:
                n_solved += 1
                cost = float(ocp_solver.get_cost())
                if best is None or cost < best["cost"]:
                    best = {
                        "cost": cost,
                        "sigs": candidate_hsigs,
                        "X": np.array([ocp_solver.get(s, "x") for s in range(N + 1)]),
                        "U": np.array([ocp_solver.get(s, "u") for s in range(N)]),
                        "internal_time": float(ocp_solver.get_stats("time_tot")),
                        "status": int(status)
                    }
        ocp_duration = time.perf_counter() - t_ocp_batch_start

        if best is None:
            print(f"ITER {iteration}: No valid OCP solutions found.")
            break

                # Time spent on this step so far, from state measurement to the end of the OCP batch
        # Artificial delay on every other iteration to test the fallback
        if TEST_DELAY_S > 0.0:
            time.sleep(TEST_DELAY_S)

        # Time spent on this step so far, from state measurement to the end of the OCP batch
        step_elapsed = time.perf_counter() - iter_start
        # Apply the control planned for the time window in which the solve finished
        if step_elapsed < STEP_TIME_LIMIT_S:
            control_idx = 0
        elif step_elapsed < 2 * STEP_TIME_LIMIT_S:
            control_idx = 1
        else:
            control_idx = 2
        control_idx = min(control_idx, best["U"].shape[0] - 1)
        print(f"ITER {iteration}: step took {step_elapsed:.3f} s, applying control {control_idx}")
        u_cmd = best["U"][control_idx, :N_players * 2].reshape(N_players, 2)

        too_close = any(
            np.linalg.norm(current_robot_locations[i] - current_robot_locations[j]) < 0.25
            for i in range(N_players) for j in range(i + 1, N_players)
        )
        emergency_stop = bool(keyboard_listener.s_toggle or too_close)
        if emergency_stop:
            u_cmd = np.zeros_like(u_cmd)

        ws_manager.publish_velocities(u_cmd)

        # Warm-start state for next iteration
        x0_full = best["X"]
        u0      = best["U"]
        previous_best_hsignatures = best["sigs"]

        best_X_reshaped = x0_full.reshape(N + 1, N_players, 3)
        current_robot_locations = best_X_reshaped[1, :, :2]
        current_theta_0         = best_X_reshaped[1, :, 2]

        total_iter_duration = time.perf_counter() - iter_start

        # ── metrics row ───────────────────────────────────────────────────────
        t_rel = time.perf_counter() - t_sim_start
        d_goal_0 = float(np.linalg.norm(
            np.asarray(curr_states[0][:2]) - np.asarray(x_goals[0], dtype=float)))
        row = [f"{iteration}", f"{t_rel:.4f}"]
        for j in range(N_players):
            row += [f"{curr_states[j][0]:.5f}",
                    f"{curr_states[j][1]:.5f}",
                    f"{curr_states[j][2]:.5f}"]
        row += [
            f"{d_goal_0:.5f}",
            "1" if d_goal_0 < 0.1 else "0",
            f"{u_cmd[0, 0]:.5f}",
            f"{u_cmd[0, 1]:.5f}",
            f"{best['cost']:.5f}",
            f"{n_attempted}",
            f"{n_solved}",
            "0",
            "1",
            "1" if emergency_stop else "0",
        ]
        f_met.write(",".join(row) + "\n")
        sigs_log = [list(s) if hasattr(s, "__iter__") and not isinstance(s, str) else s
                    for s in best["sigs"]]
        step_record = {
            "step": int(iteration),
            "t_wall": time.perf_counter() - t_sim_start,
            "states": curr_states,
            "control_applied": u_cmd.tolist(),
            "control_idx":     control_idx,
            "emergency_stop":  emergency_stop,
            "min_inter_robot_dist": float(min(
                np.linalg.norm(np.array(curr_states[i][:2]) - np.array(curr_states[j][:2]))
                for i in range(N_players) for j in range(i+1, N_players)
            )),
            "timings": {
                "warm_start": warm_start_duration,
                "homotopy":   homotopy_duration,
                "ranking":    ranking_duration,
                "ocp_batch":  ocp_duration,
                "step_total": total_iter_duration
            },
            "solver": {
                "status": best["status"],
                "cost":   best["cost"],
                "internal_time": best["internal_time"]
            },
            "method_specific": {
                "best_sigs": sigs_log,
                "n_combinations_total": len(all_combinations),
                "n_candidates_solved":  n_solved
            }
        }
        full_data["step_data"].append(step_record)
        append_jsonl(txt_path, {"type": "step", **step_record})

        if (time.perf_counter() - last_dump_time) > DUMP_INTERVAL_S:
            with open(pkl_path, "wb") as f:
                pickle.dump(full_data, f)
            f_met.flush()
            last_dump_time = time.perf_counter()

    ws_manager.stop()
    f_met.close()

    full_data["metadata"]["timestamp_end"] = time.time()
    full_data["metadata"]["steps_completed"] = len(full_data["step_data"])
    with open(pkl_path, "wb") as f:
        pickle.dump(full_data, f)
    append_jsonl(txt_path, {"type": "end",
                            "steps_completed": len(full_data["step_data"]),
                            "timestamp_end": full_data["metadata"]["timestamp_end"]})

    print(f"[{METHOD_NAME}] Instance {instance_idx}: saved {pkl_path}, {txt_path} "
          f"and {met_path}")