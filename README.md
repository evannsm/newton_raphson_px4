# NRFlow_PX4_PKG

A ROS 2 Newton-Raphson trajectory tracking controller for quadrotors. This package was created during my PhD to deploy the novel Newton-Raphson Flow control method developed mainly at Georgia Tech led by Dr. Yorai Wardi. The hardware deployment and testing was done in collaboration with Dr. Wardi by myself under the supervision of my advisor Dr. Samuel Coogan. We have compared this against the well-established NMPC technique available in my other repository [`NMPC_PX4_PKG`](https://github.com/evannsm/NMPC_PX4_PKG).


## Key Features

- **Newton-Raphson control law** — novel tracking control method developed at Georgia Tech
- **Integral CBF safety constraints** — optional barrier functions to enforce input constraints (enabled by default)
- **JAX JIT-compiled** — all control computations are JIT-compiled for real-time performance
- **PX4 integration** — publishes attitude setpoints and offboard commands via `px4_msgs`
- **Structured logging** — optional CSV logging via ROS2Logger with automatic analysis notebook generation

## Control Parameters

| Parameter | Value              | Description                               |
| --------- | ------------------ | ----------------------------------------- |
| `ALPHA`   | `[20, 30, 30, 30]` | Control gains `[x, y, z, yaw]`            |
| `USE_CBF` | `True`             | Enable integral Control Barrier Functions |

## Usage

```bash
# Source your ROS 2 workspace
source install/setup.bash

# Fly a circle in simulation
ros2 run nr_standard run_node --platform sim --trajectory circle_horz

# Fly a helix on hardware with logging
ros2 run nr_standard run_node --platform hw --trajectory helix --log

# Hover mode 3, double speed, with yaw spin
ros2 run nr_standard run_node --platform sim --trajectory hover --hover-mode 3 --double-speed --spin
```

### CLI Options

| Flag                                            | Description                       |
| ----------------------------------------------- | --------------------------------- |
| `--platform {sim,hw}`                           | Target platform (required)        |
| `--trajectory {hover,yaw_only,circle_horz,...}` | Trajectory type (required)        |
| `--hover-mode {1..8}`                           | Hover sub-mode (1-4 for hardware) |
| `--log`                                         | Enable CSV data logging           |
| `--log-file NAME`                               | Custom log filename               |
| `--double-speed`                                | 2x trajectory speed               |
| `--short`                                       | Short variant (fig8_vert)         |
| `--spin`                                        | Enable yaw rotation               |
| `--flight-period SEC`                           | Custom flight duration            |

## Dependencies

- [quad_trajectories](https://github.com/evannsm/quad_trajectories) — trajectory definitions
- [quad_platforms](https://github.com/evannsm/quad_platforms) — platform abstraction
- [ROS2Logger](https://github.com/evannsm/ROS2Logger) — experiment logging
- [px4_msgs](https://github.com/PX4/px4_msgs) — PX4 ROS 2 message definitions
- JAX / jaxlib

## Package Structure

```
nr_standard/
├── nr_standard/
│   ├── run_node.py              # CLI entry point and argument parsing
│   └── ros2px4_node.py          # ROS 2 node (subscriptions, publishers, control loop)
└── nr_standard_utils/
    ├── controller/
    │   ├── nr_standard.py       # Newton-Raphson control law
    │   └── nr_utils.py          # Dynamics, Jacobians, CBF functions
    ├── px4_utils/               # PX4 interface and flight phase management
    ├── transformations/         # Yaw adjustment utilities
    ├── main_utils.py            # Helper functions
    └── jax_utils.py             # JAX configuration
```

## Installation

```bash
# Inside a ROS 2 workspace src/ directory
git clone git@github.com:evannsm/nr_standard.git
cd .. && colcon build --symlink-install
```

## License

MIT

## Papers and Repositories

American Control Conference 2024 — [paper](https://coogan.ece.gatech.edu/papers/pdf/cuadrado2024tracking.pdf)
| [Personal repo](https://github.com/evannsm/MoralesCuadrado_ACC2024)
| [FACTSLab repo](https://github.com/gtfactslab/MoralesCuadrado_Llanes_ACC2024)

Transactions on Control Systems Technology 2025 — [paper](https://arxiv.org/abs/2508.14185)
| [Personal repo](https://github.com/evannsm/MoralesCuadrado_Baird_TCST2025)
| [FACTSLab repo](https://github.com/gtfactslab/Baird_MoralesCuadrado_TRO_2025)

Transactions on Robotics 2025
| [Personal repo](https://github.com/evannsm/MoralesCuadrado_Baird_TCST2025)
| [FACTSLab repo](https://github.com/gtfactslab/MoralesCuadrado_Baird_TCST2025)

### Related Work

- [2025_NewtonRaphson_QuadrotorComplete](https://github.com/evannsm/2025_NewtonRaphson_QuadrotorComplete)
- [Blimp_SimHardware_NR_MPC_FBL_BodyOfWork2024](https://github.com/evannsm/Blimp_SimHardware_NR_MPC_FBL_BodyOfWork2024)