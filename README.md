# Uncertainty-Responsive Safe MPC

This repository contains MATLAB code for a safe Model Predictive Fault-Tolerant Controller (MPFTC) applied to autonomous vehicle lane-keeping and obstacle avoidance. The controller is designed to handle uncertainty in predicted obstacle trajectories and adapts its braking authority in real time based on the risk level.

The vehicle model uses curvilinear (Frenet-frame) coordinates relative to a reference path, and the NLP is solved with [CasADi](https://web.casadi.org/).

---

## Quick start

1. Make sure the [dependencies](#dependencies) are installed.
2. Open MATLAB and set the working directory to the root of this repository.
3. Run:

```matlab
run_vehicle_example
```

This will simulate the closed-loop scenario (≈10 s of driving at 20 m/s with a crossing pedestrian), save the results to `data/vehicle/`, and display the plots.

---

## Dependencies

| Toolbox / Library | Purpose |
|---|---|
| MATLAB (R2020b or later recommended) | Core language |
| Control System Toolbox | Discrete-time model conversion (`c2d`) used in `utils/gen_Sval.m` |
| [CasADi](https://web.casadi.org/) ≥ 3.5 | Symbolic NLP formulation and solving (IPOPT backend) |
| [MPT3](https://www.mpt3.org/) + YALMIP | Only needed for `terminal_set.m` and `obs_obs.m` — not required to run the main example |

CasADi must be on the MATLAB path. The easiest way is to add it in `startup.m` or before running the code:

```matlab
addpath('/path/to/casadi')
```

---

## Repository structure

```
.
├── run_vehicle_example.m          % Entry point — run this
├── vehicle_safe_mpftc.m           % Main MPC solver (closed-loop simulation)
├── plot_vehicle_results.m         % Generates plots from saved results
├── gen_obstacle_predictions.m     % LPV observer design that generates obs_traj_pred.mat
├── terminal_set.m                 % Terminal set computation (requires MPT3)
├── obs_traj_pred.mat              % Predicted obstacle trajectories (input data)
├── log330.mat                     % Raw sensor log used by gen_obstacle_predictions.m
├── data/
│   └── vehicle/                   % Simulation output saved here
├── data_ECC/
│   └── vehicle/                   % Pre-computed results for reference
└── utils/
    ├── casadi_rk4.m               % RK4 integrator built with CasADi
    ├── gen_sval_list.m            % Computes safe distance bounds for each horizon step
    ├── gen_sval.m                 % LP-based reachable set bound for one horizon length
    ├── traj_matrices.m            % Builds prediction matrices for LP
    ├── gen_path.m                 % Cross-platform file path helper
    ├── draw_veh_cg_steer.m        % Vehicle body visualization
    ├── state_indexing.m           % Variable indexing helpers for NLP
    ├── constraint_indexing.m      % Constraint indexing helpers
    ├── plot_trajectories.m        % General trajectory plotting helper
    ├── tight_subplot.m            % Third-party: compact subplot layout
    └── tightfig.m                 % Third-party: remove figure whitespace
```

---

## Vehicle model

The ego vehicle is described in a curvilinear frame with seven states:

| Symbol | Description |
|---|---|
| `s` | Progress along the reference path [m] |
| `e_y` | Lateral deviation from path [m] |
| `e_psi` | Heading error [rad] |
| `delta` | Steering angle [rad] |
| `alpha` | Steering rate [rad/s] |
| `v` | Longitudinal speed [m/s] |
| `a` | Longitudinal acceleration [m/s²] |

Control inputs are acceleration request `a_req` and steering setpoint `delta_sp`. The dynamics are integrated with RK4 inside the NLP.

---

## How the controller works

At each time step the controller solves a nonlinear OCP over a horizon of `M = 100` steps (`ts = 0.1 s`). The formulation includes:

- **Obstacle constraint**: a minimum longitudinal gap to the predicted obstacle position, tightened based on the uncertainty in the prediction.
- **Safety margin (`S_val`)**: computed offline via a linear program that bounds how quickly the vehicle can decelerate from its current speed. This determines how much longitudinal clearance is needed given the remaining time to impact.
- **Fault-tolerant braking**: the maximum allowed deceleration is relaxed in proportion to the threat level, so the controller can brake harder only when needed.
- **Terminal cost**: a pre-computed LQR cost `P` that stabilises the lateral-longitudinal dynamics at the end of the horizon.

The closed-loop runs for `sim_time = 10 s` and logs states, controls, open-loop predictions, and timing.

---

## Reproducing the plots

After running `run_vehicle_example`, the plots are generated automatically. You can also re-run them independently on saved data:

```matlab
addpath(genpath('.'));
plot_vehicle_results;
```

The pre-computed results in `data_ECC/` were used in the ECC paper submission and can be loaded by pointing `plot_vehicle_results.m` to that folder instead.

---

## Citation

This code accompanies the following paper. If you use it in your work, please cite:

```bibtex
@inproceedings{quan2025uncertainty,
  title={An Uncertainty-Responsive Safe MPC for Autonomous Driving in Dynamic Environments},
  author={Quan, Ying Shuai and Falcone, Paolo and Sj{\"o}berg, Jonas},
  booktitle={2025 European Control Conference (ECC)},
  pages={2223--2228},
  year={2025},
  organization={IEEE}
}
```

---

## Notes

- First run will take longer because CasADi traces and compiles the NLP. Subsequent runs with the same problem size are faster.
- `terminal_set.m` and `obs_obs.m` are standalone scripts for design purposes and do not need to be run to reproduce the simulation results.
- Figure export (`.eps`) is commented out in `plot_vehicle_results.m`. Uncomment the `saveas` lines and set `path_fig` to your desired output folder if needed.
