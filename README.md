# Distributed Multi-Stream Beamforming in MIMO Multi-Relay Networks

This project implements a simulation of distributed multi-stream beamforming in MIMO multi-relay interference networks using MATLAB. It supports joint transmit and relay optimization using ADMM-based methods and semidefinite programming, following the methodology presented in the paper:

*Yetis, C. M., Shamaei, A., & Heath, R. W. (2019). Distributed Multi-Stream Beamforming in MIMO Multi-Relay Interference Networks.*

## Requirements

To run this code, you need to install the CVX convex optimization toolbox for MATLAB:

- **CVX**: [Download here](http://cvxr.com/cvx/download/)
- (Optional) **SeDuMi solver**: [Download here](https://github.com/sqlp/sedumi) — only required if SDPT3 is not working properly

After downloading CVX:
1. Unzip the CVX folder into your MATLAB path.
2. Run `cvx_setup` in the Matlab terminal.
3. If prompted, you can choose a solver. We recommend starting with **SDPT3**. If SDPT3 fails, install SeDuMi and re-run `cvx_setup` to select SeDuMi.

## Usage

- Run `main.m` to simulate a single channel realization using joint transmit and relay beamforming. The script prints total transmit power and displays a bar graph comparing target and achieved SINR per stream.
  
- Run `run_deliverables_demo.m` to evaluate system performance across a sweep of SNR, antenna, and relay configurations. It produces a summary plot of average and minimum SINR across all test cases and saves the results in `demo_results.mat`.
  - Please note that this sweep takes a while when using a normal number of optimization steps. As such, we've made max_iter = 1 in admm_beamforming_solver.m (down from 75), max_trials = 1 in feasibility_search.m (down from 20), and max_iter = 1 in joint_transmit_relay_beamforming.m (down from 20) if you want to try a demo of our code that runs in a shorter time span.
