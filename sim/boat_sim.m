function boat_sim()
%BOAT_SIM  Interactive position-control simulator for the GARC USV.
%   Run:  boat_sim
%   Move the sliders and every plot re-simulates immediately. Press "Play"
%   to animate the boat from above, or "Save GIF" to export the animation
%   (for the portfolio page).
%
%   HOW IT WORKS (the design, in order):
%   1. Physics: the same model as chris_position_ss.m,
%          m*v' = -d*v + F + F_dist,   x' = v
%      integrated numerically (4th-order Runge-Kutta, fixed 0.02 s step).
%   2. Actuator: the engine can only push so hard, so the commanded thrust
%      is clipped to +/- F_max. This is the one thing the transfer-function
%      analysis cannot capture, and it is why we simulate at all.
%   3. Controller: PID with the derivative on the measured velocity
%      (no "derivative kick" on a step command):
%          F = Kp*(r - x) + Ki*integral(r - x) - Kd*v
%      Anti-windup: the integral stops growing while the thrust is
%      saturated, or it would build up a huge overshoot.
%   4. Scenario: step command to r metres at t = 0, then a wind/current
%      disturbance F_dist switches on at t = 30 s.
%   5. The dashed line is the same simulation WITHOUT the thrust limit,
%      i.e. what the linear analysis predicts. Where the two lines differ,
%      saturation is changing the answer.
%
%   Works in MATLAB (R2014b+) and GNU Octave. Uses classic uicontrol
%   sliders rather than App Designer so the whole design is readable here.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'shared'));
p = params();

% ---- Slider definitions: {field, label, min, max, start value, format}
S = {
  'Kp',   'Kp  [N/m]',        0,    3000, 500,      '%.0f'
  'Kd',   'Kd  [N s/m]',      0,    5000, 1285,     '%.0f'
  'Ki',   'Ki  [N/(m s)]',    0,    200,  0,        '%.0f'
  'm',    'Mass m [kg]',      1000, 4000, p.m,      '%.0f'
  'd',    'Drag d [N s/m]',   50,   400,  p.d,      '%.0f'
  'Fd',   'Disturbance [N]', -800,  800,  p.F_dist, '%.0f'
  'r',    'Command r [m]',    1,    40,   5,        '%.1f'
  'Fmax', 'Thrust limit [N]', 500,  8000, p.F_max,  '%.0f'
};

fig = figure('Name', 'Oh B-Rudder: GARC position control', ...
             'NumberTitle', 'off', 'Color', 'w', 'Position', [60 60 1200 720]);

% ---- Axes: position, thrust, top-down view
ax.x   = axes('Parent', fig, 'Position', [0.07 0.58 0.55 0.36]);
ax.F   = axes('Parent', fig, 'Position', [0.07 0.10 0.55 0.36]);
ax.top = axes('Parent', fig, 'Position', [0.68 0.62 0.29 0.32]);

% ---- Sliders down the right-hand side
h.sl = struct(); h.txt = struct();
for k = 1:size(S, 1)
  y = 0.52 - (k-1)*0.058;
  uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.68 y+0.022 0.17 0.025], 'String', S{k,2}, ...
            'HorizontalAlignment', 'left', 'BackgroundColor', 'w');
  h.txt.(S{k,1}) = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.88 y+0.022 0.09 0.025], 'String', sprintf(S{k,6}, S{k,5}), ...
            'HorizontalAlignment', 'right', 'BackgroundColor', 'w');
  h.sl.(S{k,1}) = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.68 y 0.29 0.022], 'Min', S{k,3}, 'Max', S{k,4}, ...
            'Value', S{k,5}, 'Callback', @(~,~) update(fig));
end
uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', 'String', 'Play', ...
          'Position', [0.68 0.04 0.09 0.045], 'Callback', @(~,~) animate(fig, ''));
uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', 'String', 'Save GIF', ...
          'Position', [0.78 0.04 0.09 0.045], ...
          'Callback', @(~,~) animate(fig, fullfile(here, 'boat_sim.gif')));
uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', 'String', 'Reset', ...
          'Position', [0.88 0.04 0.09 0.045], 'Callback', @(~,~) reset_sliders(fig));

data.S = S; data.h = h; data.ax = ax; data.p = p;
set(fig, 'UserData', data);
update(fig);
end

% =========================================================================
function v = read_sliders(fig)
% Read every slider into a struct and refresh its numeric label.
data = get(fig, 'UserData');
for k = 1:size(data.S, 1)
  f = data.S{k,1};
  v.(f) = get(data.h.sl.(f), 'Value');
  set(data.h.txt.(f), 'String', sprintf(data.S{k,6}, v.(f)));
end
end

function reset_sliders(fig)
data = get(fig, 'UserData');
for k = 1:size(data.S, 1)
  set(data.h.sl.(data.S{k,1}), 'Value', data.S{k,5});
end
update(fig);
end

% =========================================================================
function out = simulate(v, use_limit)
%SIMULATE  Closed-loop boat simulation. use_limit = false skips saturation.
dt = 0.02;  t = (0:dt:90)';  N = numel(t);
t_dist = 30;                         % disturbance switches on here [s]
X = zeros(N, 2);                     % states [x, v]
F = zeros(N, 1);  z = 0;             % thrust, integral of error

for k = 1:N-1
  e = v.r - X(k,1);
  F_cmd = v.Kp*e + v.Ki*z - v.Kd*X(k,2);          % PID, D on measurement
  if use_limit
    F(k) = min(max(F_cmd, -v.Fmax), v.Fmax);       % actuator saturation
  else
    F(k) = F_cmd;
  end
  if ~use_limit || F(k) == F_cmd, z = z + e*dt; end % anti-windup (clamping)

  Fd = v.Fd*(t(k) >= t_dist);
  f = @(s) [s(2); (-v.d*s(2) + F(k) + Fd)/v.m];    % the boat's ODE
  s = X(k,:)';                                     % RK4 step
  k1 = f(s); k2 = f(s + dt/2*k1); k3 = f(s + dt/2*k2); k4 = f(s + dt*k3);
  X(k+1,:) = (s + dt/6*(k1 + 2*k2 + 2*k3 + k4))';
end
F(N) = F(N-1);
out.t = t; out.x = X(:,1); out.v = X(:,2); out.F = F; out.t_dist = t_dist;
end

% =========================================================================
function update(fig)
data = get(fig, 'UserData');
v = read_sliders(fig);
sim = simulate(v, true);       % real boat (thrust limited)
lin = simulate(v, false);      % linear-theory prediction
ax = data.ax;

% Position vs time
cla(ax.x); hold(ax.x, 'on');
plot(ax.x, lin.t, lin.x, '--', 'Color', [0.6 0.6 0.6]);
plot(ax.x, sim.t, sim.x, 'LineWidth', 2, 'Color', [0 0.35 0.7]);
plot(ax.x, [0 sim.t(end)], [v.r v.r], 'k:');
yl = get(ax.x, 'YLim');
plot(ax.x, [sim.t_dist sim.t_dist], yl, 'Color', [0.85 0.4 0.1]);
text(sim.t_dist + 1, yl(1) + 0.1*diff(yl), 'disturbance on', 'Parent', ax.x, ...
     'Color', [0.85 0.4 0.1]);
m = step_metrics(sim.t(sim.t < sim.t_dist), sim.x(sim.t < sim.t_dist), v.r);
title(ax.x, sprintf('Position   (overshoot %.0f%%, 2%% settling %.1f s, final error %.2f m)', ...
      m.overshoot_pct, m.settling_time, v.r - sim.x(end)));
ylabel(ax.x, 'x [m]'); grid(ax.x, 'on');
legend(ax.x, {'linear theory (no thrust limit)', 'simulated boat', 'command'}, ...
       'Location', 'southeast');

% Thrust vs time
cla(ax.F); hold(ax.F, 'on');
plot(ax.F, lin.t, lin.F/1e3, '--', 'Color', [0.6 0.6 0.6]);
plot(ax.F, sim.t, sim.F/1e3, 'LineWidth', 2, 'Color', [0 0.35 0.7]);
plot(ax.F, [0 sim.t(end)],  [v.Fmax v.Fmax]/1e3, 'r--');
plot(ax.F, [0 sim.t(end)], -[v.Fmax v.Fmax]/1e3, 'r--');
title(ax.F, 'Thrust command (red dashed = engine limit)');
xlabel(ax.F, 'Time [s]'); ylabel(ax.F, 'F [kN]'); grid(ax.F, 'on');

data.sim = sim; data.v = v;
set(fig, 'UserData', data);
draw_top(fig, 1);
end

% =========================================================================
function draw_top(fig, k)
% Top-down view of the boat at time index k.
data = get(fig, 'UserData'); sim = data.sim; v = data.v; a = data.ax.top;
L = data.p.L;  B = data.p.B;
hull = [-L/2 -B/2; L/3 -B/2; L/2 0; L/3 B/2; -L/2 B/2];   % pointed bow
xmin = min([0; sim.x]) - L;  xmax = max([v.r; sim.x]) + L;

cla(a); hold(a, 'on');
fill([xmin xmax xmax xmin], [-6 -6 6 6], [0.85 0.93 1], 'EdgeColor', 'none', 'Parent', a);
plot(a, v.r, 0, 'o', 'MarkerSize', 12, 'MarkerFaceColor', [1 0.55 0], 'MarkerEdgeColor', 'k');
fill(hull(:,1) + sim.x(k), hull(:,2), [0.25 0.25 0.3], 'Parent', a);
if sim.t(k) >= sim.t_dist && v.Fd ~= 0         % disturbance arrow
  dir = sign(v.Fd);
  quiver(a, sim.x(k) - dir*L, 4, dir*L*0.8, 0, 0, 'Color', [0.85 0.4 0.1], ...
         'LineWidth', 2, 'MaxHeadSize', 2);
end
axis(a, [xmin xmax -6 6]); axis(a, 'equal'); set(a, 'YTick', []);
xlim(a, [xmin xmax]);
title(a, sprintf('Top view   t = %4.1f s   x = %5.2f m', sim.t(k), sim.x(k)));
xlabel(a, 'x [m]   (orange buoy = target)');
end

% =========================================================================
function animate(fig, gif_file)
% Play the stored simulation. With gif_file set, also write each frame.
data = get(fig, 'UserData');
idx = 1:10:numel(data.sim.t);        % every 0.2 s of sim time
for j = 1:numel(idx)
  if ~ishandle(fig), return; end
  draw_top(fig, idx(j));
  drawnow;
  if isempty(gif_file)
    pause(0.03);
  else
    frame = getframe(fig);
    [im, map] = rgb2ind(frame.cdata, 128);
    if j == 1
      imwrite(im, map, gif_file, 'gif', 'LoopCount', Inf, 'DelayTime', 0.05);
    else
      imwrite(im, map, gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
  end
end
if ~isempty(gif_file), fprintf('Saved %s\n', gif_file); end
end
