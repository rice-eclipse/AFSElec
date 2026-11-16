%RUN_AFS_FINTABSIM Fly the fin-tab rocket with and without active control.
%   Builds AFS_FinTabSim.slx if it is missing, runs a controlled flight and a
%   passive (tabs locked) flight in the same wind, prints a summary and
%   plots the comparison. Edit afs_params.m to change the vehicle.

here = fileparts(mfilename('fullpath'));
addpath(here);
mdl = 'AFS_FinTabSim';
if ~isfile(fullfile(here, [mdl '.slx']))
    build_AFS_FinTabSim();
end
load_system(fullfile(here, [mdl '.slx']));

P = afs_params();
Ppassive = P;
Ppassive.ctrl.enable = 0;

in(1) = Simulink.SimulationInput(mdl);
in(1) = in(1).setVariable('P', P);
in(2) = Simulink.SimulationInput(mdl);
in(2) = in(2).setVariable('P', Ppassive);
outs = sim(in, 'ShowProgress', 'off');

R(1) = afs_unpack(outs(1), 'Active fin tabs');
R(2) = afs_unpack(outs(2), 'Passive (tabs locked)');

fprintf('\n%-24s %10s %10s %10s %12s %12s %12s\n', 'Case', 'Apogee m', 't_apo s', 'Mach max', 'Drift m', 'Tilt@apo deg', 'Roll RMS d/s');
for k = 1:2
    r = R(k);
    fprintf('%-24s %10.0f %10.2f %10.2f %12.0f %12.1f %12.1f\n', r.name, r.apogee, r.t(end), ...
        max(r.mach), r.drift, r.tiltApo, r.rollRateRms);
end
OR = afs_import_ork(fullfile(here, 'AFS_rocket.ork'));
s = OR.sim.summary;
fprintf('%-24s %10.0f %10.2f %10.2f   (OpenRocket''s own simulation)\n', 'OpenRocket', ...
    s.maxaltitude, s.timetoapogee, s.maxmach);
fprintf('Max tab deflection (active): %.1f deg\n\n', max(abs(R(1).delta(:)))*180/pi);

afs_plot_results(R, P, OR);
