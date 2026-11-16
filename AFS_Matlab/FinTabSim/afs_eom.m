function [xdot, onRail] = afs_eom(P, x, F_B, M_B, t)
%AFS_EOM Rigid-body 6DOF equations of motion with a launch-rail constraint.
%   While the rocket is on the rail it can only slide along the rail axis
%   (and cannot sink below the pad); after rail exit it is free in 6DOF.
r = x(1:3); v = x(4:6); q = x(7:10); w = x(11:13);
C = afs_quat2dcm(q/norm(q));
[m, ~, I] = afs_mass_props(P, t);
[~, ~, ~, ~, g] = afs_atmosphere(P, r(3));

aI = C*F_B/m + [0; 0; -g];

d = P.rail.dir;
onRail = (r.'*d) < P.rail.length;
if onRail
    aI   = max(aI.'*d, 0)*d;
    wdot = zeros(3, 1);
    w    = zeros(3, 1);
else
    wdot = I\(M_B - cross(w, I*w));
end

% Quaternion kinematics with a gentle normalization term
Om = [-q(2) -q(3) -q(4);
       q(1) -q(4)  q(3);
       q(4)  q(1) -q(2);
      -q(3)  q(2)  q(1)];
qdot = 0.5*Om*w + 1.0*(1 - q.'*q)*q;

xdot = [v; aI; qdot; wdot];
end
