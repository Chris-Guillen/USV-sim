function ref_line(v)
%REF_LINE  Dashed horizontal line at y = v on the current axes.
%   Same idea as yline (MATLAB R2018b+), but also works in Octave.
xl = xlim; plot(xl, [v v], 'k--');
end
