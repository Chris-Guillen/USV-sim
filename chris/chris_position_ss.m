%% Chris: state-space model, Bode analysis, and P vs PD position control
% Oh B-Rudder (MAE 3260 final group work). Model of the GARC USV's forward
% (surge) position:
%
%       m*x'' = -d*x' + F + F_dist
%
% Run section by section (Ctrl+Enter in MATLAB) and read the comments: each
% section says WHAT it does and WHY that step is in the design.
% Figures are saved to ./figures.

clear; close all; clc;
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'shared'));          % params.m, step_metrics.m
if exist('OCTAVE_VERSION', 'builtin'), pkg load control; end
figdir = fullfile(here, 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

p = params();
m = p.m;  d = p.d;
fprintf('m = %.0f kg, d = %.1f N*s/m, F_max = %.0f N\n', m, d, p.F_max);

%% 1. Build the state-space model  x' = A x + B u,  y = C x + D u
% WHY: state space turns one 2nd-order ODE into two 1st-order ODEs. That is
% the form simulators (ode45, lsim) integrate, and it scales to multi-input,
% multi-output models later (e.g. adding heading).
%
% Choose states that are physical and measurable:
%   x1 = position x   [m]
%   x2 = velocity x'  [m/s]
% Then:
%   x1' = x2                         (definition of velocity)
%   x2' = -(d/m) x2 + (1/m) F        (Newton's 2nd law divided by m)
% Inputs u = [F; F_dist]. Both enter the same way (both are forces), so B
% gets two identical columns. Output y = position.
A = [0  1;
     0 -d/m];
B = [0    0;
     1/m  1/m];
C = [1 0];
D = [0 0];
sys = ss(A, B, C, D);

% For control design we usually look at one input at a time:
G  = ss(A, B(:,1), C, 0);   % thrust F -> position x  (the "plant")
Gd = ss(A, B(:,2), C, 0);   % disturbance F_dist -> position x

%% 2. Check against Laya's transfer function
% WHY: two people derived the same physics two ways (Laplace vs state
% space). If they don't match, one of you has an error. This check is a
% cheap, convincing thing to show in the report.
%
% Laplace of m*x'' + d*x' = F  ->  X(s)/F(s) = 1 / (m s^2 + d s)
s = tf('s');
G_laya = 1/(m*s^2 + d*s);
G_from_ss = tf(G);

disp('Transfer function from state space:'); G_from_ss
disp('Laya''s transfer function:');          G_laya

% Poles of the TF must equal eigenvalues of A.
fprintf('eig(A)       = %s\n', mat2str(eig(A)', 4));
fprintf('poles(G_laya) = %s\n', mat2str(pole(G_laya)', 4));
% Expect 0 (integrator: thrust -> velocity -> position) and -d/m = -1/tau.
% tau = m/d is the velocity time constant (about 12 s).
diff_check = norm(freqresp(G_from_ss - G_laya, 1));  % should be ~0
fprintf('|G_ss - G_laya| at 1 rad/s = %.2e  (should be ~0)\n\n', diff_check);

%% 3. Open-loop Bode plot of the plant
% WHY: the Bode plot shows how the boat responds to thrust that oscillates
% at each frequency. Read three things off it:
%   - Low frequency: slope -20 dB/dec from the integrator (pole at 0).
%   - Corner at w = d/m (about 0.08 rad/s): drag stops mattering above it.
%   - High frequency: slope -40 dB/dec, phase -> -180 deg (pure mass).
% Phase approaching -180 deg is why P-only control will be poorly damped.
w = logspace(-3, 1, 400);
figure('Name', 'Open-loop plant');
bode(G, w); grid on;
title('Open-loop plant X(s)/F(s) (thrust to position)');
print(fullfile(figdir, '01_plant_bode.png'), '-dpng', '-r150');

%% 4. P control: loop gain, margins, step response
% Controller: F = Kp*(r - x). Loop gain L(s) = Kp*G(s).
% WHY margins: gain margin (GM) = how much more gain before instability;
% phase margin (PM) = how much extra lag before instability. PM also
% predicts damping: roughly zeta = PM/100 for PM below 60 deg.
Kp = 500;                         % [N/m] 500 N of thrust per metre of error
L_P = Kp*G;
[GM_P, PM_P, ~, wc_P] = margin(L_P);
fprintf('P:  Kp = %g N/m  ->  PM = %.1f deg at %.3f rad/s\n', Kp, PM_P, wc_P);
% Closed loop: zeta = d / (2*sqrt(Kp*m)). It is small, so expect big overshoot.
zeta_P = d/(2*sqrt(Kp*m));
fprintf('    predicted zeta = %.3f\n', zeta_P);

figure('Name', 'Loop gain, P');
margin(L_P); grid on;
print(fullfile(figdir, '02_loopgain_P_margins.png'), '-dpng', '-r150');

%% 5. PD control: add derivative action to buy damping
% Controller: F = Kp*e + Kd*e'. C(s) = Kp + Kd*s.
% WHY: derivative action reacts to velocity, like adding extra drag that
% only exists in the controller. The closed loop becomes
%   m s^2 + (d + Kd) s + Kp = 0  ->  zeta = (d + Kd) / (2*sqrt(Kp*m))
% so we can pick Kd for a target damping ratio. In the Bode plot, the zero
% at s = -Kp/Kd adds up to +90 deg of phase near crossover, which is what
% raises the phase margin.
zeta_target = 0.7;
Kd = 2*zeta_target*sqrt(Kp*m) - d;   % [N*s/m]
fprintf('PD: Kp = %g N/m, Kd = %.0f N*s/m  (zeta target %.1f)\n', Kp, Kd, zeta_target);
% Real derivative terms need a low-pass filter (noise). Use a pole 10x
% faster than the zero: C = Kp + Kd*s/(Tf*s + 1).
Tf = (Kd/Kp)/10;
C_PD = Kp + Kd*s/(Tf*s + 1);
L_PD = C_PD*G;
[GM_PD, PM_PD, ~, wc_PD] = margin(L_PD);
fprintf('    PM = %.1f deg at %.3f rad/s\n\n', PM_PD, wc_PD);

figure('Name', 'Loop gain, PD');
margin(L_PD); grid on;
print(fullfile(figdir, '03_loopgain_PD_margins.png'), '-dpng', '-r150');

%% 6. Side-by-side step responses (5 m position command)
% WHY: this is the headline plot of the section. Same Kp, so any change
% comes only from derivative action.
%
% Design choice: put the derivative on the MEASUREMENT, not the error:
%   F = Kp*(r - x) - Kd*x'
% If the derivative acted on the error, a step in r would have an infinite
% slope at t = 0 and command a huge thrust spike ("derivative kick"). On
% the measurement, the closed-loop poles are identical (same damping), but
% there is no extra zero, so overshoot is lower and the thrust stays sane.
% The loop gain, and so the margins in sections 4-5, are unchanged.
r = 5;                                % [m] reference step (sized so Kp*r < F_max)
T_P  = feedback(L_P, 1);              % r -> x, P
T_PD = minreal(Kp*feedback(G, C_PD)); % r -> x, PD with derivative on measurement
t = linspace(0, 120, 6000)';
y_P  = r*step(T_P,  t);
y_PD = r*step(T_PD, t);
mP  = step_metrics(t, y_P,  r);
mPD = step_metrics(t, y_PD, r);

figure('Name', 'P vs PD step', 'Position', [100 100 900 380]);
subplot(1,2,1);
plot(t, y_P, 'LineWidth', 1.5); hold on; ref_line(r);
title(sprintf('P: Kp = %g  (OS %.0f%%, Ts %.0f s)', Kp, mP.overshoot_pct, mP.settling_time));
xlabel('Time [s]'); ylabel('Position x [m]'); grid on;
subplot(1,2,2);
plot(t, y_PD, 'LineWidth', 1.5); hold on; ref_line(r);
title(sprintf('PD: Kp = %g, Kd = %.0f  (OS %.0f%%, Ts %.0f s)', Kp, Kd, mPD.overshoot_pct, mPD.settling_time));
xlabel('Time [s]'); ylabel('Position x [m]'); grid on;
print(fullfile(figdir, '04_step_P_vs_PD.png'), '-dpng', '-r150');

fprintf('%-4s %10s %10s %12s\n', '', 'OS [%]', 'Tr [s]', 'Ts(2%) [s]');
fprintf('%-4s %10.1f %10.2f %12.1f\n', 'P',  mP.overshoot_pct,  mP.rise_time,  mP.settling_time);
fprintf('%-4s %10.1f %10.2f %12.1f\n\n', 'PD', mPD.overshoot_pct, mPD.rise_time, mPD.settling_time);

%% 7. Reality check: does the controller ask for more thrust than exists?
% WHY: linear analysis assumes any force is available. The engine tops out
% at F_max (~3.6 kN). If the commanded force exceeds that, the real boat is
% slower than the plot. Say so in the report; it shows engineering judgment.
U_P  = feedback(Kp, G);              % r -> F, P:  Kp / (1 + Kp G)
U_PD = minreal(Kp - C_PD*T_PD);      % r -> F, PD: F = Kp r - C_PD x
u_P  = r*step(U_P,  t);
u_PD = r*step(U_PD, t);
fprintf('Peak thrust: P = %.0f N, PD = %.0f N, limit F_max = %.0f N\n', ...
        max(abs(u_P)), max(abs(u_PD)), p.F_max);

%% 8. Disturbance rejection: 200 N step of wind/current
% x/F_dist = G / (1 + C G). Final value with P: F_dist/Kp (an offset that
% never goes away). PD has the same offset, since derivative action does
% nothing at steady state. Isaac's integral term is what removes it.
S_P  = feedback(G, Kp);
S_PD = feedback(G, C_PD);
xd_P  = p.F_dist*step(S_P,  t);
xd_PD = p.F_dist*step(S_PD, t);
figure('Name', 'Disturbance');
plot(t, xd_P, t, xd_PD, 'LineWidth', 1.5); grid on;
legend('P', 'PD', 'Location', 'southeast');
xlabel('Time [s]'); ylabel('Position error [m]');
title(sprintf('Response to %g N disturbance step (offset = F_{dist}/K_p = %.2f m)', p.F_dist, p.F_dist/Kp));
print(fullfile(figdir, '05_disturbance_P_vs_PD.png'), '-dpng', '-r150');

