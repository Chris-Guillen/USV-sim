function p = params()
%PARAMS  Shared GARC boat parameters for every section of the project.
%   p = params() returns a struct. EVERY script in the repo should call this
%   instead of typing numbers, so all four sections use identical values.
%   Derivations and sources: see the group parameter sheet.

%% Published specs (BlackSea / Militarnyi / AutoNocion)
p.L      = 4.8;      % length [m]
p.B      = 1.75;     % beam [m]
p.m      = 2177;     % full-load displacement [kg]  (4,800 lb)
p.P      = 149e3;    % engine power [W]             (200 hp)
p.v_max  = 20.6;     % top speed [m/s]              (40 kn)
p.v_cru  = 11.3;     % cruise speed [m/s]           (~41 km/h)

%% Estimated: position (surge) model   m*x'' = -d*x' + F + F_dist
p.eta    = 0.5;                  % assumed overall propulsive efficiency [-]
p.F_max  = p.eta*p.P/p.v_max;    % thrust limit [N]  (~3.6 kN)
p.d      = p.F_max/p.v_max;      % linear drag coefficient [N*s/m]  (~175)
p.F_dist = 200;                  % baseline disturbance step (wind + current) [N]

%% Estimated: heading (yaw) model, first-order Nomoto   T*r' + r = K*delta
p.Iz        = p.m*(p.L^2 + p.B^2)/12;  % yaw inertia, uniform box [kg*m^2]
p.K_yaw     = 1.5;                      % Nomoto gain [1/s]
p.T_yaw     = 1.5;                      % Nomoto time constant [s]
p.delta_max = deg2rad(30);              % steering limit [rad]
end
