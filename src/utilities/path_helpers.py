from utilities.cphyton_set import get_filtered_homotopic_trajectories
import numpy as np

def compute_robot_homotopies_updated(i, robot_locations, x_goals, d_spacing, o_radius=0.25):
    robot = robot_locations[i]
    x_goal = x_goals[i]
    obstacles = np.array([r for j, r in enumerate(robot_locations) if j != i])
    print(f"Robot {i+1} planning from {robot} to {x_goal} with {len(obstacles)} obstacles: {obstacles.tolist()}")
    result = get_filtered_homotopic_trajectories(robot, x_goal, obstacles, 0.20, d_spacing[i], plot=False, save=False)
    print(f"Robot {i+1} found {len(result['trajectories'])} homotopy trajectories")
    return f"Robot_{i+1}", [(hsig, traj) for hsig, traj in result["trajectories"].items()]

def pad_traj_to_length(traj, target_length):
    traj = np.asarray(traj, dtype=np.float64)
    if traj.ndim != 2 or traj.shape[1] != 2:
        raise ValueError(f"Expected shape (T, 2), got {traj.shape}")
    current_len = traj.shape[0]
    if current_len >= target_length:
        padded_traj = traj[:target_length]
    else:
        last_row = traj[-1]
        num_pad = target_length - current_len
        pad_array = np.tile(last_row, (num_pad, 1))
        padded_traj = np.vstack([traj, pad_array])
    return padded_traj