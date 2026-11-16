function [F_B, M_B, aux] = afs_aero(P, x, delta, wind, t)
%AFS_AERO Aerodynamic + propulsive forces and moments in the body frame.
%   x     : state [r(3); v(3); q(4); w(3)]
%   delta : tab deflections (rad), one per fin
%   wind  : wind velocity, ENU (m/s)
%   aux   : [alpha; beta; mach; qbar; aoaTotal; thrust; mass; xCg]
v = x(4:6); q = x(7:10); w = x(11:13);
q = q/norm(q);
C = afs_quat2dcm(q);

[m, xCg, ~, T] = afs_mass_props(P, t);
[rho, a] = afs_atmosphere(P, x(3));

vAirB = C.'*(v - wind);                 % air-relative velocity, body frame
V     = max(norm(vAirB), 1e-3);
mach  = V/a;
qbar  = 0.5*rho*V^2;
S     = P.geo.Sref;

% Compressibility factor for fin lift (linear theory, capped near Mach 1)
if mach < 1
    kc = min(1/sqrt(max(1 - mach^2, 1e-6)), P.aero.compMax);
else
    kc = min(1/sqrt(max(mach^2 - 1, 1e-6)), P.aero.compMax);
end

% Axial force (power-on / power-off tables from OpenRocket)
mc = min(mach, P.aero.machTable(end));
if T > 0
    CA = interp1(P.aero.machTable, P.aero.CAonTable, mc);
else
    CA = interp1(P.aero.machTable, P.aero.CAoffTable, mc);
end
F_B = [-qbar*S*CA*sign(vAirB(1)); 0; 0];
M_B = zeros(3, 1);

% Nose/body normal force acting at the body CP
Fn  = -qbar*S*P.aero.CNaBody*[0; vAirB(2); vAirB(3)]/V;
rN  = [-(P.geo.xCpBody - xCg); 0; 0];
F_B = F_B + Fn;
M_B = M_B + cross(rN, Fn);

% Each fin: local flow (includes body rotation -> pitch/yaw/roll damping),
% fin cant (roll disturbance) and the tab deflection (control)
for i = 1:P.geo.nFins
    [r, tv] = afs_fin_geometry(P, i, xCg);
    vLoc = vAirB + cross(w, r);
    aLoc = (vLoc.'*tv)/V;                % local angle of attack seen by the fin
    CN   = kc*(P.aero.CNaFin*(P.aero.finCant(i) - aLoc) + P.aero.CNdTab*delta(i));
    Fi   = qbar*S*CN*tv;
    F_B  = F_B + Fi;
    M_B  = M_B + cross(r, Fi);
end

% Thrust along the (slightly misaligned) motor axis, acting at the nozzle
if T > 0
    ey = P.prop.misalign(1); ez = P.prop.misalign(2);
    Ft = T*[cos(ey)*cos(ez); sin(ez); -sin(ey)];
    rT = [-(P.geo.L - xCg); 0; 0];
    F_B = F_B + Ft;
    M_B = M_B + cross(rT, Ft);
end

alpha = atan2(vAirB(3), vAirB(1));
beta  = asin(max(min(vAirB(2)/V, 1), -1));
aoa   = acos(max(min(vAirB(1)/V, 1), -1));
aux = [alpha; beta; mach; qbar; aoa; T; m; xCg];
end
