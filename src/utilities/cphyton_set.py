import numpy as np
import matplotlib.pyplot as plt
from math import atan2, sin, cos, pi
from collections import defaultdict
import os
import random
from scipy.interpolate import splprep, splev
# from computer_h_signature import compute_h_signature
import utilities.add_points as add_points
import utilities.h_signature as h_signature
import utilities.graph_builder as graph_builder
import utilities.path_length_cpp as path_length_cpp 
import utilities.slopes as slopes  # import the compiled extension
def get_filtered_homotopic_trajectories(x_start, x_goal, obstacles, o_radius, d_spacing, plot=True, save=True):
    adj, node_labels = graph_builder.generate_graph(x_start, x_goal, obstacles, o_radius)
    all_paths = defaultdict(list)
    def dfs(current, path_nodes, path_points):
        if current == "Goal":
            hs = h_signature.compute_h_signature(path_points, obstacles)
            all_paths[hs].append(list(path_nodes))
            return
        for neighbor in adj[current]:
            if neighbor not in path_nodes:
                dfs(neighbor, path_nodes + [neighbor], path_points + [node_labels[neighbor]])
    dfs("Start", ["Start"], [node_labels["Start"]])
    shortest_paths = path_length_cpp.shortest_paths(node_labels, all_paths)
        # Filter paths based on homotopy signature
    dx, dy = x_goal[0] - x_start[0], x_goal[1] - x_start[1]
    m_slope = np.arctan2(dy, dx)
    angle_range = np.pi / 2
    lower_bound = m_slope - angle_range
    upper_bound = m_slope + angle_range
    filtered_shortest_paths = {
        hsig: path for hsig, path in shortest_paths.items()
        if slopes.path_slopes_within_range([node_labels[n] for n in path], lower_bound, upper_bound)
    }
    # B-spline smoothing with added intermediate points and spacing ~d_spacing meters
    bspline_trajectories = {}
    for hsig, path in filtered_shortest_paths.items():
        pts = np.array([node_labels[n] for n in path])
        # Add intermediate points
        pts_dense = add_points.addIntermediatePoints(pts, n_points=5)
        pts_dense = np.array(pts_dense).T
        if pts_dense.shape[1] < 4:
            bspline_trajectories[hsig] = pts_dense.T  # Not enough points for spline
            continue
        try:
            # Fit spline
            tck, _ = splprep(pts_dense, s=0.2, k=2)
            # Sample spline finely to compute cumulative arc length
            u_fine = np.linspace(0, 1, 1000)
            fine_pts = np.array(splev(u_fine, tck)).T
            # Compute cumulative distances along fine sample
            deltas = np.diff(fine_pts, axis=0)
            seg_lengths = np.linalg.norm(deltas, axis=1)
            cum_length = np.insert(np.cumsum(seg_lengths), 0, 0)
            # Desired spacing (d_spacing meters)
            spacing = d_spacing
            desired_distances = np.arange(0, cum_length[-1], spacing)
            # Interpolate to find corresponding parameter values 'u' for those distances
            u_new = np.interp(desired_distances, cum_length, u_fine)
            # Evaluate spline at spaced parameters
            spaced_pts = np.array(splev(u_new, tck)).T
            bspline_trajectories[hsig] = spaced_pts
        except Exception as e:
            # fallback to original points if spline fails
            bspline_trajectories[hsig] = pts_dense.T
        # If no trajectories found, fallback to straight line
    if not bspline_trajectories:
        line = np.array([x_start, x_goal])
        # Interpolate intermediate points along the line
        vec = np.array(x_goal) - np.array(x_start)
        total_len = np.linalg.norm(vec)
        if total_len == 0:
            spaced_pts = np.array([x_start])
        else:
            num_points = max(int(np.floor(total_len / d_spacing)) + 1, 2)
            t_vals = np.linspace(0, 1, num_points)
            spaced_pts = np.outer(1 - t_vals, x_start) + np.outer(t_vals, x_goal)
        bspline_trajectories["direct_line"] = spaced_pts
    if plot or save:
        output_dir = "filtered_homotopy_paths_bspline"
        os.makedirs(output_dir, exist_ok=True)
        for i, (hsig, smoothed_pts) in enumerate(bspline_trajectories.items()):
            fig, ax = plt.subplots()
            ax.plot(x_start[0], x_start[1], 'go', label='Start')
            ax.plot(x_goal[0], x_goal[1], 'ro', label='Goal')
            for c in obstacles:
                circle = plt.Circle(c, o_radius, color='r', fill=False)
                ax.add_patch(circle)
            for n, p in node_labels.items():
                ax.plot(p[0], p[1], 'bo' if n not in ["Start", "Goal"] else 'ko')
            # for a, b in edges:
            #     p1, p2 = node_labels[a], node_labels[b]
            #     ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'gray', alpha=0.3)
            ax.plot(smoothed_pts[:, 0], smoothed_pts[:, 1], 'b-', linewidth=3, label=f"Smoothed Path {i+1}")
            ax.set_aspect('equal')
            ax.legend()
            plt.grid(True)
            plt.title(f"B-spline Path {i+1} (H-signature: {hsig})")
            plt.tight_layout()
            if save:
                filename = os.path.join(output_dir, f"path_{i+1}_hsig_{i}.png")
                plt.savefig(filename)
                plt.close()
            elif plot:
                plt.show()
    return {
        "trajectories": bspline_trajectories,
        "h_signatures": list(bspline_trajectories.keys()),
        "paths": bspline_trajectories
    }
