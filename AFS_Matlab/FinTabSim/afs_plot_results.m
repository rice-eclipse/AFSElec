function fig = afs_plot_results(R, P, OR)
%AFS_PLOT_RESULTS Compare flights unpacked by afs_unpack.
%   R(1) is drawn as the primary (active) case, the rest as dashed references.
%   Pass the struct from afs_import_ork as OR to overlay OpenRocket's altitude.
if nargin < 3
    OR = [];
end
r2d = 180/pi;
fig = figure('Name', 'AFS fin-tab flight', 'Color', 'w', 'Position', [80 60 1300 820]);
tl = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Fin-tab controlled rocket: 6DOF simulation');
sty = {'-', '--', ':'};

% Trajectory
ax = nexttile(tl, [2 1]); hold(ax, 'on'); grid(ax, 'on');
for k = 1:numel(R)
    plot3(ax, R(k).x(:, 1), R(k).x(:, 2), R(k).alt, sty{k}, 'LineWidth', 1.5, 'DisplayName', R(k).name);
end
xlabel(ax, 'East (m)'); ylabel(ax, 'North (m)'); zlabel(ax, 'Up (m)');
title(ax, 'Trajectory'); legend(ax, 'Location', 'northwest'); view(ax, 35, 20); axis(ax, 'equal');

ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
for k = 1:numel(R)
    plot(ax, R(k).t, R(k).alt, sty{k}, 'LineWidth', 1.2, 'DisplayName', R(k).name);
end
if ~isempty(OR)
    D = OR.sim.data;
    keep = D.Time <= R(1).t(end) + 0.5;
    plot(ax, D.Time(keep), D.Altitude(keep), 'k:', 'LineWidth', 1.5, 'DisplayName', 'OpenRocket');
end
ylim(ax, [0 inf]);
yyaxis(ax, 'right');
plot(ax, R(1).t, R(1).mach, '-', 'LineWidth', 1.0, 'DisplayName', 'Mach (active)');
ylabel(ax, 'Mach (active)'); ylim(ax, [0 inf]);
yyaxis(ax, 'left'); ylabel(ax, 'Altitude (m)'); xlabel(ax, 't (s)'); title(ax, 'Altitude & Mach');
legend(ax, 'Location', 'southeast');

ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
for k = 1:numel(R)
    plot(ax, R(k).t, R(k).roll*r2d, sty{k}, 'LineWidth', 1.2);
end
stairs(ax, R(1).t, R(1).phiCmd*r2d, 'k:', 'LineWidth', 1.0);
xlabel(ax, 't (s)'); ylabel(ax, 'deg'); title(ax, 'Roll angle (dotted = command)');
legend(ax, [{R.name}, {'command'}], 'Location', 'best');
ylim(ax, [-180 max(360, max(R(1).roll*r2d) + 20)]);

ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
for k = 1:numel(R)
    plot(ax, R(k).t, R(k).rollRate*r2d, sty{k}, 'LineWidth', 1.2);
end
xlabel(ax, 't (s)'); ylabel(ax, 'deg/s'); title(ax, 'Roll rate');

ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
for k = 1:numel(R)
    plot(ax, R(k).t, R(k).tilt*r2d, sty{k}, 'LineWidth', 1.2);
end
yline(ax, 90 - P.rail.elev*r2d, 'k:', 'rail');
xlabel(ax, 't (s)'); ylabel(ax, 'deg'); title(ax, 'Body-axis tilt from vertical');

ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
for k = 1:numel(R)
    plot(ax, R(k).t, R(k).aoa*r2d, sty{k}, 'LineWidth', 1.2);
end
xlabel(ax, 't (s)'); ylabel(ax, 'deg'); title(ax, 'Total angle of attack');
ylim(ax, [0 15]);

ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
plot(ax, R(1).t, R(1).delta*r2d, 'LineWidth', 1.1);
yline(ax, [-1 1]*P.ctrl.defMax*r2d, 'k:');
xlabel(ax, 't (s)'); ylabel(ax, 'deg'); title(ax, 'Tab deflections (active)');
legend(ax, compose('tab %d', 1:P.geo.nFins), 'Location', 'best');

ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
plot(ax, R(1).t, R(1).qbar/1e3, 'LineWidth', 1.2);
xlabel(ax, 't (s)'); ylabel(ax, 'kPa'); title(ax, 'Dynamic pressure (active)');
end
