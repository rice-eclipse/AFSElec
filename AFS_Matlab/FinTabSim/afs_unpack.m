function R = afs_unpack(out, name)
%AFS_UNPACK Turn a Simulink.SimulationOutput from AFS_FinTabSim into plain arrays.
%   Each logged signal becomes an N-by-width matrix on its own time vector.
R.name  = name;
R.t     = out.x.Time;
R.x     = tsmat(out.x);
R.aux   = tsmat(out.aux);
R.delta = tsmat(out.delta);
R.tCtrl = out.ctrl.Time;
R.ctrl  = tsmat(out.ctrl);
R.tCmd  = out.deltaCmd.Time;
R.deltaCmd = tsmat(out.deltaCmd);
R.phiCmd = tsmat(out.phiCmd);
R.wind  = tsmat(out.wind);

R.alt   = R.x(:, 3);
R.mach  = R.aux(:, 3);
R.qbar  = R.aux(:, 4);
R.aoa   = R.aux(:, 5);
R.rollRate = R.x(:, 11);
R.roll  = cumtrapz(R.t, R.rollRate);        % true roll angle about the body axis

% Body-axis tilt from vertical
n = numel(R.t);
R.tilt = zeros(n, 1);
for k = 1:n
    C = afs_quat2dcm(R.x(k, 7:10).'/norm(R.x(k, 7:10)));
    R.tilt(k) = acos(max(min(C(3, 1), 1), -1));
end

R.apogee  = R.alt(end);
R.drift   = norm(R.x(end, 1:2));
R.tiltApo = R.tilt(end)*180/pi;
mask = R.t > 1;
R.rollRateRms = sqrt(mean(R.rollRate(mask).^2))*180/pi;
end

function M = tsmat(ts)
d = ts.Data;
if ndims(d) == 3
    M = permute(d, [3 1 2]);
    M = reshape(M, size(M, 1), []);
else
    M = d;
end
end
