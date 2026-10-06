#!/usr/bin/env python3
import subprocess
import numpy as np
import math
from utilities.config_files import N_players
ROBOT_NAMES = [f"turtlebot3_burger_{i+1}" for i in range(N_players)]
def reset_world():
    subprocess.run(["gz", "world", "-r"], check=True)
    print("Gazebo world reset.")

def move_model(name, x, y, yaw_deg=0.0, z=0.0):
    yaw_rad = math.radians(yaw_deg)
    subprocess.run([
        "gz", "model", "-m", name,
        "-x", str(x),
        "-y", str(y),
        "-z", str(z),
        "-Y", str(yaw_rad)
    ], check=True)

def move_robots(start_positions, goal_positions):
    """
    start_positions : (N,2) array
    goal_positions  : (N,2) array
    """
    assert start_positions.shape == goal_positions.shape
    assert start_positions.shape[0] == len(ROBOT_NAMES)

    for i, name in enumerate(ROBOT_NAMES):
        x, y = start_positions[i]
        gx, gy = goal_positions[i]

        yaw_deg = math.degrees(math.atan2(gy - y, gx - x))
        move_model(name, x, y, yaw_deg)

