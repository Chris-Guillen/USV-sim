# Oh B-Rudder: Controlling an Uncrewed Surface Vessel

MAE 3260 (System Dynamics) final group work, Cornell.
Models and controllers for the forward position and heading of a GARC-class
uncrewed surface vessel, plus an interactive MATLAB simulator.

## Quick start (MATLAB or GNU Octave)

```matlab
cd sim
boat_sim          % interactive slider simulator
```

```matlab
cd chris
chris_position_ss % state space, Bode, P vs PD, writes figures/
```

Octave needs the control package for the analysis scripts: `pkg install -forge control` once.

## Layout

| Folder | Owner | Contents |
| --- | --- | --- |
| `shared/` | everyone | `params.m` (the ONE source of numbers), helper functions |
| `laya/` | Laya | Equations of motion, open-loop transfer functions, parameter estimation |
| `chris/` | Chris | State-space model, Bode / margins, P vs PD position control |
| `andy/` | Andy | Heading block diagrams, disturbance and sensor-filter analysis |
| `isaac/` | Isaac | PI / PID position design, integral action, K_I stability range |
| `sim/` | Chris, Isaac | `boat_sim.m` interactive simulator |

## Rules for the repo

1. Never type a boat parameter into your script. Call `p = params();` and use `p.m`, `p.d`, and so on.
   If a number needs to change, change it in `shared/params.m` and tell the group.
2. Every script adds the shared folder with
   `addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'shared'));`
3. Save figures to your own `figures/` folder as PNG so they can go straight into the report and portfolio pages.

## Model summary

Position: `m x'' = -d x' + F + F_dist`, so `X/F = 1/(m s^2 + d s)`.
Heading (first-order Nomoto): `T r' + r = K delta`, `r = psi'`, so `Psi/delta = K / (s (T s + 1))`.
Parameter values and their sources: see the group parameter sheet.
