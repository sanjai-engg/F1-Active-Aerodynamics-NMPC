# Model and Reproducibility Notes

These notes record inconsistencies visible in the supplied scripts and presentation. They are intentionally explicit so the repository does not overstate what has been implemented.

## 1. The supplied files contain multiple controller/model stages

| Component | Supplied configuration | Notes |
|---|---|---|
| CasADi NMPC script | 10 states, 4 controls; `N = 30`, `dt = 0.05 s`; RK2 transcription; IPOPT | Principal uploaded implementation. The control horizon in this script is the full `N` decision sequence. |
| Day 2 MATLAB NMPC | 6 states, 4 outputs, 3 controls; prediction horizon 30; control horizon 5 | Earlier reduced-order bicycle-style model with a simplified active-aero force law. |
| `setup_nmpc.m` | 5 states, 3 outputs, 3 controls | Its dimensions do not match the currently supplied six-state `NMPC_StateFcn` and four-output `NMPC_OutputFcn`. |
| Day 3 MATLAB NMPC | 10 states, 10 outputs, 4 controls; prediction horizon 30; control horizon 3 | Uses `VehicleDynamics7DOF.m`; distinct from the CasADi script's local `calc_dynamics` implementation. |

Do not describe these as one common controller implementation. The report's NMPC strategy slide states a prediction horizon of 50 and a 0.05 s sampling time (2.5 s preview); the latest CasADi script and Day 3 runner use a horizon of 30 (1.5 s at the same sample time).

## 2. Track geometry discrepancy

`Track_layout.m` sets the first apex radius to **85 m** and the second to **300 m**. The project presentation (page 5) describes the second apex as **340 m**. The generated MAT file is retained as supplied. Resolve this discrepancy before claiming the track geometry exactly matches the presentation or the real Suzuka 130R.

## 3. Airfoil polar inputs

The supplied `SeligS1223.txt` and `Eppler_E423.txt` datasets are included at the repository root. They are headerless seven-column numeric tables; the supplied scripts use columns 1–3 as angle of attack, lift coefficient, and drag coefficient. `Airfoil_Setup.m` has been changed to read zero header lines so it does not discard valid data rows. The CasADi script still generates simplified placeholder tables if either expected filename is absent from the MATLAB current working directory. The supplied data have not been independently checked against an external source or test conditions.

## 4. Differences in the two 7-DOF model implementations

The CasADi script's local `calc_dynamics` function calculates longitudinal and lateral load-transfer terms and applies drive torque to the rear axle with brake torque distributed according to the configured bias. In contrast, `VehicleDynamics7DOF.m` uses static axle loads plus aerodynamic load and divides the commanded torque equally across all four wheels. These are materially different driveline/load-transfer assumptions. The MATLAB model should not be described as having the same RWD/brake-bias implementation as the CasADi model without revising it.

The `VehicleDynamics7DOF.m` file does not currently calculate dynamic longitudinal/lateral load transfer, despite that feature being described in the presentation's vehicle/tire-dynamics slide (page 3). `TireSlipConstraints.m` is not registered in `Run_Day3_RealPlant.m`, so its slip-angle inequalities are not active in the supplied Day 3 run.

## 5. Missing supporting assets

- The `Vehicle_Cornering2.slx` model required by `cornering.m` is included under `src/supplementary_analysis/`. Unreal Engine visualization assets and additional Simulink visualization model files mentioned in the presentation (pages 12–14) are not included.

## 6. Results and validation

The supplied presentation includes simulation plots (pages 9–11), but it does not provide a complete machine-readable run log or tabulated quantitative validation metrics. This package does not invent numerical performance results. The scripts were not run in MATLAB during packaging; the README therefore makes no claim of an end-to-end verified build.
