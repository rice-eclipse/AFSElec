function P = afs_params(orkFile)
%AFS_PARAMS Parameters for the fin-tab controlled rocket simulation.
%   P = AFS_PARAMS() builds the parameter struct for AFS_FinTabSim from the
%   OpenRocket design AFS_rocket.ork next to this file.
%   P = AFS_PARAMS(orkFile) uses another .ork file.
%
%   What comes from OpenRocket (re-save the .ork after running its sim):
%     geometry, fin planform/position, launch rail and wind settings, thrust
%     curve, mass/CG/pitch-inertia history, and drag coefficient vs. Mach.
%     The normal-force slopes and CPs are computed here with Barrowman's
%     equations from the same geometry; they agree with OpenRocket's CP.
%   What OpenRocket does not know and is set by hand below:
%     fin tabs, servos, sensors, controller, build misalignments, roll
%     inertia (OpenRocket only stores it to 3 decimals).
%
%   Frames
%     Inertial : East-North-Up (ENU), origin at the launch pad.
%     Body     : x out the nose, y/z completing a right-handed set.
%                Fin i sits at body roll angle finAngle(i) about +x.
%   Quaternion q = [w x y z] rotates body vectors into the inertial frame.
%   All units SI. Stations are measured aft from the nose tip.
here = fileparts(mfilename('fullpath'));
if nargin < 1
    orkFile = fullfile(here, 'AFS_rocket.ork');
end
OR = afs_import_ork(orkFile);
D  = OR.sim.data;
C  = OR.sim.conditions;
G  = OR.geo;
F  = G.fin;

%% Simulation
P.sim.tStop        = 20;      % s, hard stop (model also stops at apogee)
P.sim.maxStep      = 2e-3;    % s, solver max step
P.sim.stopAtApogee = true;

%% Environment (from the OpenRocket simulation conditions)
P.env.g0        = 9.80665;
P.env.Re        = 6371e3;
P.env.hSite     = C.siteAlt;
% OpenRocket's wind direction is where the wind comes FROM (0 = North, pi/2 = East)
P.env.windENU   = -C.windSpeed*[sin(C.windDir); cos(C.windDir); 0];
P.env.gustTau   = 1.0;                                  % s, gust filter time constant
gustSigma       = C.windTurb*C.windSpeed;               % m/s, OpenRocket turbulence intensity
P.env.gustPower = 2*P.env.gustTau*gustSigma^2;          % white-noise power giving that sigma
P.env.gustSeed  = [11 12 13];

%% Launch rail
P.rail.length = C.railLength;          % m
P.rail.elev   = pi/2 - C.railAngle;    % rad from horizontal
P.rail.azim   = C.railDir;             % rad, 0 = North

%% Geometry
P.geo.d        = G.d;
P.geo.L        = G.L;
P.geo.Sref     = pi*G.d^2/4;
P.geo.nFins    = F.n;
P.geo.finAngle = F.rotation + (0:F.n - 1)*2*pi/F.n;
[xFin, yMac, CNaFins, xNose, CNaNose] = barrowman(G);
P.geo.xFin     = xFin;                 % fin CP station
P.geo.rFin     = G.d/2 + yMac;         % radius of fin spanwise CP (mean aero chord)
P.geo.xCpBody  = xNose;                % nose normal-force CP station

%% Mass properties: OpenRocket's own history over the burn
tB = burnout_time(OR);
k  = [true; diff(D.Time) > 0] & D.Time <= tB + 1e-9;
P.mass.tTable   = D.Time(k).';
P.mass.mTable   = D.Mass(k).';
P.mass.xCgTable = D.CGLocation(k).';
P.mass.IyyTable = D.LongitudinalMomentOfInertia(k).';
% Roll inertia: OpenRocket rounds it to 0.001 kg m^2, so it is set here.
% Estimate from the parts list: tubes + coupler + nose ~1.1e-3, servos and
% motor ~0.4e-3 (fins have a 0 kg mass override in the .ork).
P.mass.IxxFull  = 1.55e-3;
P.mass.IxxEmpty = 1.45e-3;

%% Propulsion (OpenRocket's thrust samples for this motor)
P.prop.tTable   = P.mass.tTable;
P.prop.TTable   = D.Thrust(k).';
P.prop.TTable(end) = 0;
P.prop.tBurn    = tB;
P.prop.misalign = deg2rad([0.10; -0.08]);  % rad, thrust angle about body y and z (build error)

%% Aerodynamics
P.aero.CNaBody = CNaNose;
P.aero.CNaFin  = CNaFins/(F.n/2);      % per fin; F.n/2 fins act in each plane
% Axial-force coefficient vs. Mach from OpenRocket, after rail exit
tRod = event_time(OR, 'launchrod', 0.1);
tApo = event_time(OR, 'apogee', D.Time(end));
on  = D.Time > tRod & D.Thrust > 0;
off = D.Time > tB & D.Time < tApo - 0.5;
P.aero.machTable  = linspace(0, max(D.MachNumber(D.Time < tApo))*1.05, 8);
P.aero.CAonTable  = bin_fit(D.MachNumber(on),  D.AxialDragCoefficient(on),  P.aero.machTable);
P.aero.CAoffTable = bin_fit(D.MachNumber(off), D.AxialDragCoefficient(off), P.aero.machTable);
P.aero.finCant = deg2rad([0.10 0.15 0.05 0.12]) + F.cant; % rad, fin misalignment (build error)
P.aero.compMax = 1.8;                  % cap on the Prandtl-Glauert style factor

%% Fin tabs (not in OpenRocket; set to the real tab design)
P.aero.tauTab  = 0.30;                 % tab effectiveness: fraction of the fin's CNa per rad of tab
P.aero.CNdTab  = P.aero.tauTab*P.aero.CNaFin;

%% Fin-tab servos
P.servo.tau     = 0.025;               % s, first-order response
P.servo.rateMax = deg2rad(500);        % rad/s, slew limit (~0.12 s/60 deg)
P.servo.defMax  = deg2rad(15);         % rad, mechanical deflection limit

%% Sensors (sampled at the controller rate)
P.sens.Ts        = 0.01;               % s, 100 Hz flight loop
P.sens.gyroBias  = deg2rad([0.3; -0.2; 0.1]);
P.sens.gyroNoise = (deg2rad(0.05))^2*P.sens.Ts;
P.sens.gyroSeed  = [21 22 23];
P.sens.qbarNoise = 15;                 % Pa, 1-sigma dynamic-pressure estimate noise

%% Flight controller
P.ctrl.enable    = 1;                  % 0 = tabs locked at zero (passive rocket)
P.ctrl.tEnable   = tRod + 0.05;        % s, earliest time the tabs may move
P.ctrl.qbarMin   = 200;                % Pa (~18 m/s), below this the tabs are held at zero
P.ctrl.qbarFloor = 400;                % Pa, floor used in the gain schedule
P.ctrl.defMax    = deg2rad(12);        % rad, commanded limit (inside the mechanical stop)
% Roll: PID on roll angle, gains in angular-acceleration units
P.ctrl.roll.Kp   = 400;
P.ctrl.roll.Ki   = 400;
P.ctrl.roll.Kd   = 25;
P.ctrl.roll.iMax = 1.5;                % rad s
% Pitch/yaw: hold P.ctrl.targetDir (set Kp = Ki = 0 for rate damping only)
P.ctrl.py.Kp     = 30;
P.ctrl.py.Ki     = 20;
P.ctrl.py.Kd     = 6;
P.ctrl.py.iMax   = deg2rad(10);
% Roll program: roll-angle command steps (s, rad)
P.ctrl.rollProg.t   = [0 2 4.5];
P.ctrl.rollProg.phi = deg2rad([0 90 0]);
% Nominal inertia and CG the controller assumes (mid-flight values)
P.ctrl.Inom      = diag([1.5e-3, mean(P.mass.IyyTable([1 end])), mean(P.mass.IyyTable([1 end]))]);
P.ctrl.xCgNom    = mean(P.mass.xCgTable([1 end]));

%% Derived quantities
P.rail.dir = [cos(P.rail.elev)*sin(P.rail.azim);
              cos(P.rail.elev)*cos(P.rail.azim);
              sin(P.rail.elev)];
P.ctrl.B0   = afs_tab_effectiveness(P, P.ctrl.xCgNom);
P.ctrl.Binv = pinv(P.ctrl.B0);
P.ctrl.targetDir = [0; 0; 1];          % pitch/yaw hold target (ENU): vertical

%% Initial state  x = [r_ENU(3); v_ENU(3); q(4); w_body(3)]
P.x0 = [0; 0; 0; 0; 0; 0; quat_from_x_axis(P.rail.dir); 0; 0; 0];
end

%% ======================================================================
function [xFin, yMac, CNaFins, xNose, CNaNose] = barrowman(G)
% Barrowman normal-force slopes (per rad, ref. area pi d^2/4) and CP stations.
F = G.fin;
R = G.d/2;
a = F.root; b = F.tip; s = F.span; m = F.sweep;
lm = hypot(s, m + b/2 - a/2);                        % mid-chord line length
CNaFins = (4*F.n*(s/G.d)^2/(1 + sqrt(1 + (2*lm/(a + b))^2))) * (1 + R/(s + R));
xFin = F.xLE + m*(a + 2*b)/(3*(a + b)) + (a + b - a*b/(a + b))/6;
yMac = s/3*(a + 2*b)/(a + b);
CNaNose = 2;
switch lower(G.noseShape)
    case 'conical', xNose = 0.666*G.noseLength;
    case 'ogive',   xNose = 0.466*G.noseLength;
    otherwise,      xNose = 0.5*G.noseLength;
end
end

function tB = burnout_time(OR)
tB = event_time(OR, 'burnout', NaN);
if isnan(tB)
    D = OR.sim.data;
    tB = D.Time(find(D.Thrust > 0, 1, 'last') + 1);
end
end

function t = event_time(OR, type, default)
t = default;
k = find(strcmp({OR.sim.events.type}, type), 1);
if ~isempty(k)
    t = OR.sim.events(k).time;
end
end

function y = bin_fit(x, v, edges)
% Mean of v in Mach bins centred on edges; empty bins take the nearest value.
y = nan(size(edges));
h = (edges(2) - edges(1))/2;
for k = 1:numel(edges)
    in = abs(x - edges(k)) <= h;
    if any(in)
        y(k) = mean(v(in));
    end
end
ok = ~isnan(y);
y = interp1(edges(ok), y(ok), edges, 'nearest', 'extrap');
end

function q = quat_from_x_axis(d)
% Shortest rotation taking body +x to the inertial direction d.
x = [1; 0; 0];
axis = cross(x, d);
s = norm(axis);
ang = atan2(s, dot(x, d));
if s < 1e-12
    q = [cos(ang/2); 0; 0; sin(ang/2)];   % identity, or 180 deg about z if d = -x
else
    q = [cos(ang/2); sin(ang/2)*axis/s];
end
end
