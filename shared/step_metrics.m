function s = step_metrics(t, y, y_final)
%STEP_METRICS  Overshoot, rise time and 2% settling time of a step response.
%   Works in MATLAB and Octave (Octave's control package has no stepinfo).
%   t, y    : time and response vectors
%   y_final : the value the response should settle to
y = y(:); t = t(:);
s.overshoot_pct = max(0, (max(y) - y_final)/abs(y_final)*100);

i10 = find(y >= 0.1*y_final, 1);   % rise time: 10% -> 90%
i90 = find(y >= 0.9*y_final, 1);
if isempty(i10) || isempty(i90), s.rise_time = NaN;
else, s.rise_time = t(i90) - t(i10); end

outside = find(abs(y - y_final) > 0.02*abs(y_final));  % last time outside 2% band
if isempty(outside), s.settling_time = 0;
elseif outside(end) == numel(t), s.settling_time = Inf;  % never settled in window
else, s.settling_time = t(outside(end) + 1); end
end
