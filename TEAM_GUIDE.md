# Team guide: what each section needs to produce

Due dates: parameters agreed **Mon Nov 30**, work sessions **Nov 30, Dec 2, Dec 4**,
report + portfolio link **Mon Dec 7**. Each person should show 3-4 course skills,
with clear plots (axis labels with units, legends, titles) and your own write-up.
The course does not allow AI tools to write the report, so the code and plots
here are tools; the explanations in the report have to be yours.

All sections use `shared/params.m`. Start each script with:

```matlab
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'shared'));
if exist('OCTAVE_VERSION','builtin'), pkg load control; end
p = params();
```

## Laya: equations of motion and open-loop plants (`laya/`)

Skills: ODEs, Laplace transforms, transfer functions, pole/zero analysis, open-loop step and Bode response, parameter estimation.

- [ ] Free-body diagrams for surge and yaw, and the two ODEs.
- [ ] Laplace-transform each to get `X/F = 1/(m s^2 + d s)` and `Psi/delta = K/(s(Ts+1))`.
- [ ] Pole-zero map for each; explain the pole at 0 (integrator) and the real pole (`-d/m`, `-1/T`).
- [ ] Open-loop step of velocity (`V/F = 1/(m s + d)`) showing top speed `F_max/d` and time constant `m/d`.
- [ ] Bode plots of both plants with the corner frequency marked.
- [ ] Write up how each parameter in the parameter sheet was estimated.

Note: the position plant has an integrator, so its open-loop step in position grows forever. Show the velocity step for "steady state".

## Chris: state space and P vs PD (`chris/`)

Skills: state space, Bode / stability margins, feedback control, step response.
Already started: `chris/chris_position_ss.m` builds A, B, C, D, checks against Laya's TF, plots margins, and compares P with PD.

## Andy: heading block diagrams (`andy/`)

Skills: block diagrams, closed-loop transfer functions, disturbance rejection, filtering.

- [ ] Open-loop block diagram: rudder -> Nomoto yaw dynamics -> heading.
- [ ] Closed-loop diagram with a PD (or PID) heading controller, a disturbance torque input, and a sensor block `1/(tau_f s + 1)` on the compass.
- [ ] Derive, by block-diagram algebra, `Psi/Psi_ref` and `Psi/disturbance`. Check each with `feedback()` in MATLAB.
- [ ] Step plots: open loop vs closed loop, disturbance rejection, filtered vs unfiltered sensor.
- [ ] Sweep `tau_f` (e.g. 0.05, 0.2, 0.5 s) and show the trade-off: noise rejection against added lag (lower phase margin, more overshoot).
- [ ] Respect `p.delta_max`: check the peak rudder command against +/- 30 deg.

## Isaac: PI / PID position control (`isaac/`)

Skills: controller design to specs, steady-state error, integral action, stability range.

- [ ] Pick specs (for example, overshoot < 10% and 2% settling < 15 s) and design PI and PID gains to meet them.
- [ ] Show steady-state error to a constant disturbance `p.F_dist`: P/PD leave `F_dist/Kp`, PI/PID remove it.
- [ ] Find the range of `K_I` that keeps the loop stable (Routh-Hurwitz on `m s^3 + (d+Kd) s^2 + Kp s + Ki`, so `Ki < (d+Kd) Kp / m`). Confirm with `pole()` or a root locus.
- [ ] Use `sim/boat_sim.m` (the Ki slider) to show integrator windup when the thrust limit is hit, and why the simulator's anti-windup matters.
- [ ] Check simulator traces against `step()` results for the same gains (the dashed "linear theory" line).

## Shared checks before Dec 4

- [ ] Everyone's plots use the same `params.m` values.
- [ ] Chris's `tf(ss)` matches Laya's TF; Isaac's PID with `Ki = 0` matches Chris's PD.
- [ ] Each person can name their 3-4 skills in one line for the report.
