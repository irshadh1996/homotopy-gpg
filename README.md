# Homotopy-Guided Potential Games for Congestion-Aware Navigation

Implementation of the paper **"Homotopy-Guided Potential Games for Congestion-Aware Navigation"** (IEEE Robotics and Automation Letters, 2026). [[arXiv]](https://arxiv.org/abs/2604.13708)

We address the multi-agent motion planning problem where interactions, collisions, and congestion co-exist. Conventional game-theoretic planners capture interactions among agents but often converge to conservative, congested equilibria. Homotopy planners, on the other hand, can explore topologically distinct paths, but lack mechanisms to account for the interdependence of agents' future actions. We propose a unified framework that leverages homotopy classes as structured strategy sets within a receding-horizon setup. At each planning stage, a deterministic homotopy planner generates topologically distinct paths for each agent, conditioned on the joint configuration. To avoid intractable growth of candidate paths, we propose a simple heuristic filtering step that selects a top-K subset of the most suitable congestion-free joint strategies to ensure computational tractability. These serve as initializations for a potential game that enforces homotopy-consistent constraints and yields a generalized open-loop Nash equilibrium (OLNE), with penalties discouraging abrupt strategy shifts in a receding-horizon setting.

## Note on platforms

The experiments in the paper were conducted on ClearPath Jackal robots, in simulation and on hardware. This repository provides a Gazebo setup with three TurtleBot3 Burger robots, and up to five can be used. Since the planner models both platforms with unicycle kinematics, it applies to either by setting the corresponding control limits. The physical parameters here are tuned for the TurtleBot3, so numerical results will differ from those reported in the paper.

## Requirements

- Python 3.10 (tested)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation) with Gazebo 11
- [TurtleBot3 simulation packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) (`turtlebot3_gazebo`, `turtlebot3_description`)
- [rosbridge_server](http://wiki.ros.org/rosbridge_suite) (`sudo apt install ros-noetic-rosbridge-server`)
- [acados](https://docs.acados.org/installation/) with its Python interface (`acados_template`)

The planner communicates with ROS through rosbridge (websocket, port 9090) and resets/positions the robots with Gazebo's `gz` command-line tool (Gazebo server, port 11345). It can therefore run outside the ROS environment, e.g. on the host while ROS Noetic and Gazebo run in a Docker container with host networking, as long as the `gz` command is available on the machine running the planner.

## Installation

```bash
git clone https://github.com/irshadh1996/homotopy-gpg.git
cd homotopy-gpg

# Python packages
pip install -r requirements.txt

# acados Python interface (from your acados installation)
pip install -e <acados_root>/interfaces/acados_template

# Build the C++ extensions (placed in src/utilities/)
sudo apt install python3-dev
python setup.py build_ext --inplace
```

ROS package (in your catkin workspace):

```bash
ln -s <path-to>/homotopy-gpg/ros ~/catkin_ws/src/homotopy_gpg_gazebo
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

## Usage

**1. Start the simulation** (Gazebo with three TurtleBot3 robots and rosbridge):

```bash
export TURTLEBOT3_MODEL=burger
roslaunch homotopy_gpg_gazebo custom_3.launch
```

**2. Run the planner** (from the repository root):

```bash
python src/main.py
```

This runs the default three-robot scenario. The robots are moved to their start positions automatically. To run a custom scenario:

```bash
python src/main.py '<starts as JSON>' '<goals as JSON>' <instance_idx>
# example
python src/main.py '[[-1.5,0],[1.5,0],[0,-1.5]]' '[[1.5,0],[-1.5,0],[0,1.5]]' 1
```

A keyboard listener provides a manual emergency stop (see `src/utilities/keyboard_listener.py`).

Results are written to `results/homotopy_guided_games/`:

- `log/data_<idx>.txt` — per-step log (JSON lines)
- `pkl/data_<idx>.pkl` — full run data
- `scenario_<idx>/metrics_agent0.txt` — robot states per step, used for post-processing

**3. Post-process** a scenario (trajectory plot and metric statistics):

```bash
python analysis/post_process.py                                    # default: scenario_manual
python analysis/post_process.py results/homotopy_guided_games/scenario_1
```

This prints completion time, path length, minimum clearance, and dwell time in the conflict band, and saves `trajectories.png` in the scenario folder.

## Adapting to another robot

- Reference velocity: `vel_ref_0` in `src/main.py`
- Control limits and robot radius: solver model in `src/utilities/acados_solver_param.py`
- Robot names and ROS topics: `src/utilities/config_files.py`

## Adding robots

Up to 5 robots are supported. To add robots:

- Set `N_players` in `src/utilities/config_files.py`
- Add the spawn entries for the new robots in `ros/launch/custom_3.launch`
- Provide N start and goal positions (command-line arguments, or the defaults in `src/main.py`)

## Repository structure

```
homotopy-gpg/
├── src/
│   ├── main.py                  # planner entry point
│   └── utilities/               # solver model, homotopy computation, ranking,
│                                # rosbridge communication, C++ extension sources
├── analysis/
│   └── post_process.py          # trajectory plot and metric statistics
├── ros/                         # ROS package homotopy_gpg_gazebo (launch file)
├── setup.py                     # builds the C++ extensions
└── requirements.txt
```

## Citation

If you use this code, please cite:

```bibtex
@article{imran2026homotopy,
  title     = {Homotopy-Guided Potential Games for Congestion-Aware Navigation},
  author    = {Imran, Mohammed IIS and Peters, Lasse and Khayyat, Michael and Arrigoni, Stefano and Braghin, Francesco and Ferranti, Laura},
  journal   = {IEEE Robotics and Automation Letters},
  year      = {2026},
  publisher = {IEEE}
}
```

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).
