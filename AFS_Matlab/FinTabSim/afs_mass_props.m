function [m, xCg, I, T] = afs_mass_props(P, t)
%AFS_MASS_PROPS Thrust and mass properties at time t.
%   Mass, CG and pitch inertia follow OpenRocket's history (P.mass tables),
%   held at their burnout values afterwards. Roll inertia is blended between
%   P.mass.IxxFull and IxxEmpty by propellant fraction.
tt = P.mass.tTable;
tc = min(max(t, tt(1)), tt(end));
m   = interp1(tt, P.mass.mTable, tc);
xCg = interp1(tt, P.mass.xCgTable, tc);
Iyy = interp1(tt, P.mass.IyyTable, tc);
f   = (m - P.mass.mTable(end))/max(P.mass.mTable(1) - P.mass.mTable(end), eps);
Ixx = P.mass.IxxEmpty + f*(P.mass.IxxFull - P.mass.IxxEmpty);
I   = diag([Ixx Iyy Iyy]);
T = 0;
if t >= P.prop.tTable(1) && t < P.prop.tTable(end)
    T = interp1(P.prop.tTable, P.prop.TTable, t);
end
end
