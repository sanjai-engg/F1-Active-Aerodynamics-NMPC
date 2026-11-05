# Optimal Autonomous Control of an F1 Car with Active Aerodynamics

**Topic:** Nonlinear Model Predictive Control (NMPC) for a simplified double-apex track inspired by the 130R corner at Suzuka.

This repository documents the team's ME5670 course project. The original course presentation PDF was intentionally excluded from this repository. 
## Project overview

The project explores closed-loop vehicle control with steering, longitudinal wheel torque, and continuously variable front/rear wing angles of attack as control inputs. The principal implementation is a MATLAB script using **CasADi** and **IPOPT**. Its local vehicle model uses a 10-state Frenet-frame representation: progress along the track, lateral and heading errors, longitudinal/lateral velocity, yaw rate, and four wheel angular speeds.

The main implementation includes:

- **NMPC** with a finite prediction horizon, a nonlinear programming solver, warm-started trajectories, and actuator slew-rate penalties/limits.
- A **7-DOF double-track vehicle model** with four wheel-speed states, combined-slip tyre forces, and wheel-torque actuation.
- **Active aerodynamics**, with front and rear wing angle-of-attack commands contributing to downforce and aerodynamic drag.
- **Frenet-frame path tracking** using track arc length, lateral error, heading error, and a sampled curvature preview.
- A generated **double-apex reference track** and MATLAB plots for speed, steering, wing commands, lateral deviation, torque, and trajectory.

This repository contains multiple development stages, not one single interchangeable implementation. See the model notes below before choosing an entry point.

## Repository structure

```text
.
├── README.md
├── 130R_DoubleApex_Trajectory.mat
├── Eppler_E423.txt
├── SeligS1223.txt
├── docs/
│   └── MODEL_AND_REPRODUCIBILITY_NOTES.md
└── src/
    ├── aerodynamics/
    │   └── Airfoil_Setup.m
    ├── casadi_nmpc/
    │   └── NEW_F1_nmpc_activeAero_v5.m
    ├── matlab_7dof_model/
    │   ├── Run_Day3_RealPlant.m
    │   ├── TireSlipConstraints.m
    │   └── VehicleDynamics7DOF.m
    ├── matlab_nmpc_prototype/
    │   ├── NMPC_OutputFcn.m
    │   ├── NMPC_StateFcn.m
    │   ├── Run_Day2_UnitTest.m
    │   └── setup_nmpc.m
    ├── supplementary_analysis/
    │   ├── Vehicle_Cornering2.slx
    │   ├── cornering.m
    │   └── pacejka.m
    └── track_generation/
        └── Track_layout.m
```

## Main entry point: CasADi/IPOPT NMPC

From the repository root, first run `addpath(genpath(pwd))`, then run `src/casadi_nmpc/NEW_F1_nmpc_activeAero_v5.m` in MATLAB after configuring the MATLAB interface for **CasADi** and ensuring **IPOPT** is available through that installation. The script uses the track data in `130R_DoubleApex_Trajectory.mat` when that file is available; otherwise it creates a fallback curvature profile.

The current source filename says `v5`, while the script header and comments describe **v7**. The filename is preserved from the upload to avoid silently renaming the submitted source.

### Airfoil polar data

The repository includes the supplied `SeligS1223.txt` and `Eppler_E423.txt` numeric polar tables at the repository root. Both files are headerless and contain seven numeric columns. The main CasADi script and `Airfoil_Setup.m` use the first three columns: angle of attack, lift coefficient (`C_L`), and drag coefficient (`C_D`). The remaining columns are retained from the supplied data but are not used by these scripts.

Run the scripts with the repository root as MATLAB's current working directory so the relative filenames resolve. With both files present there, the main CasADi script loads the supplied tables instead of generating its simplified fallback tables. The source data have been included as supplied; their provenance and experimental conditions should be checked before treating them as independently validated airfoil measurements.

## Other MATLAB implementations

- `src/matlab_nmpc_prototype/Run_Day2_UnitTest.m` is an earlier six-state, three-input MATLAB NMPC prototype using `NMPC_StateFcn.m` and `NMPC_OutputFcn.m`.
- `src/matlab_nmpc_prototype/setup_nmpc.m` is an earlier setup script. Its dimensions (5 states / 3 outputs) do not match the currently uploaded state/output functions (6 states / 4 outputs); it is retained as a development artifact, not advertised as a ready-to-run entry point.
- `src/matlab_7dof_model/Run_Day3_RealPlant.m` uses MATLAB's `nlmpc` workflow with `VehicleDynamics7DOF.m` and the 130R MAT data. `TireSlipConstraints.m` is included as a candidate custom constraint function, but the supplied Day 3 runner does not register it as the controller's custom inequality constraint.
- `src/track_generation/Track_layout.m` regenerates the double-apex track table. The supplied MAT file is kept as a separate input data file.
- `src/supplementary_analysis/cornering.m` calls a Simulink model named `Vehicle_Cornering2`; that `.slx` model was not included in the supplied files. `pacejka.m` contains supplementary coursework calculations/plots rather than the main controller.

## Dependencies

- MATLAB.
- **CasADi MATLAB interface** and **IPOPT** for the principal CasADi NMPC script.
- MATLAB **Model Predictive Control Toolbox** for the `nlmpc` / `nlmpcmove` prototype scripts.
- The included `src/supplementary_analysis/Vehicle_Cornering2.slx` model is required by `cornering.m`.

For scripts that load the track MAT file by its basename, keep the MATLAB current working directory at the repository root. Add the repository tree to the MATLAB path with `addpath(genpath(pwd))` before running scripts in subfolders.

No MATLAB/Octave runtime was available during preparation of this repository package, so the uploaded scripts have been organised and statically inspected but **have not been executed end-to-end in this environment**.

## Scope and interpretation

This is a simulation and coursework implementation, not a validated F1 vehicle model or a real-time controller deployed on a physical vehicle. Parameters and simplified aerodynamic/tyre relations are model assumptions. Interpret the project plots and any results reported in the original course presentation within those limitations.

## Project team

**Team XTorque**

- Ashish Sam
- Frazel Fernandes
- Joel J Nellikkunnel
- Mukhil M
- Sanjai V


## Acknowledgements

Generative AI assistance (ChatGPT) was used during repository preparation for code-review discussions, debugging suggestions, repository-structure organisation, and documentation drafting. AI-generated suggestions were used as assistance, not as independent verification. The project team remains responsible for reviewing the source code, validating simulations, and checking the technical content and results presented in this repository.
